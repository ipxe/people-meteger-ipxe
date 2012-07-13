/*
 * Copyright (C) 2012 Daniel Wyatt <Daniel.Wyatt@gmail.com>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/malloc.h>
#include <ipxe/pci.h>
#include <ipxe/mii.h>
#include "bnx2.h"
#include "bnx2_fw.h"

/** @file
 *
 * bnx2 network driver
 *
 */

static uint32_t bnx2_reg_read_indirect ( struct bnx2_nic *bnx2,
										 uint32_t offset ) {
	uint32_t value;
	pci_write_config_dword ( bnx2->pci,
							 BNX2_PCICFG_REG_WINDOW_ADDRESS,
							 offset );
	pci_read_config_dword ( bnx2->pci, BNX2_PCICFG_REG_WINDOW, &value );
	return value;
}

static void bnx2_reg_write_indirect ( struct bnx2_nic *bnx2,
									  uint32_t offset,
									  uint32_t value ) {
	pci_write_config_dword ( bnx2->pci,
							 BNX2_PCICFG_REG_WINDOW_ADDRESS,
							 offset );
	pci_write_config_dword ( bnx2->pci, BNX2_PCICFG_REG_WINDOW, value );
}

static void bnx2_shmem_init ( struct bnx2_nic *bnx2 ) {
	uint32_t value;
	value = bnx2_reg_read_indirect ( bnx2, BNX2_SHM_HDR_SIGNATURE );
	if ( ( value & BNX2_SHM_HDR_SIGNATURE_SIG_MASK ) == BNX2_SHM_HDR_SIGNATURE_SIG )
		bnx2->shmem = bnx2_reg_read_indirect ( bnx2, BNX2_SHM_HDR_ADDR_0 );
	else
		bnx2->shmem = BNX2_HOST_VIEW_SHMEM_BASE;
}

static uint32_t bnx2_shmem_read ( struct bnx2_nic *bnx2, uint32_t offset ) {
	return bnx2_reg_read_indirect ( bnx2, bnx2->shmem + offset );
}

static void bnx2_shmem_write ( struct bnx2_nic *bnx2, uint32_t offset,
							   uint32_t value ) {
	bnx2_reg_write_indirect ( bnx2, bnx2->shmem + offset, value );
}

static void bnx2_set_mac_address ( struct bnx2_nic *bnx2,
								   struct net_device *netdev ) {
	uint32_t value;
	value = bnx2_shmem_read ( bnx2, BNX2_PORT_HW_CFG_MAC_UPPER );
	netdev->hw_addr[0] = ( uint8_t ) ( value >> 8 );
	netdev->hw_addr[1] = ( uint8_t ) value;
	value = bnx2_shmem_read ( bnx2, BNX2_PORT_HW_CFG_MAC_LOWER );
	netdev->hw_addr[2] = ( uint8_t ) ( value >> 24 );
	netdev->hw_addr[3] = ( uint8_t ) ( value >> 16 );
	netdev->hw_addr[4] = ( uint8_t ) ( value >> 8 );
	netdev->hw_addr[5] = ( uint8_t ) value;

	/* We use the lower 3 bytes as the back-off seed */
	value = netdev->hw_addr[5] + netdev->hw_addr[4] + netdev->hw_addr[3];
	value &= BNX2_EMAC_BACKOFF_SEED_EMAC_BACKOFF_SEED;
	writel ( value, bnx2->regs + BNX2_EMAC_BACKOFF_SEED );
}

static int bnx2_alloc_mem ( struct bnx2_nic *bnx2 ) {
	bnx2->status_blk = malloc_dma ( sizeof ( struct status_block ),
									sizeof ( struct status_block ) );
	bnx2->tx_ring.tx_desc_ring = malloc_dma ( sizeof ( struct bnx2_tx_bd ) * TX_DESC_CNT,
											  sizeof ( struct bnx2_tx_bd ) );
	if ( !bnx2->status_blk || !bnx2->tx_ring.tx_desc_ring ) {
		/* These are zeroed earlier so this is safe */
		free_dma ( bnx2->status_blk, sizeof ( struct status_block ) );
		free_dma ( bnx2->tx_ring.tx_desc_ring, sizeof ( struct bnx2_tx_bd ) );
		return -ENOMEM;
	}
	memset ( bnx2->status_blk, 0, sizeof ( struct status_block ) );
	memset ( bnx2->tx_ring.tx_desc_ring, 0, sizeof ( struct bnx2_tx_bd ) * TX_DESC_CNT );
	return 0;
}

static void bnx2_free_mem ( struct bnx2_nic *bnx2 ) {
	free_dma ( bnx2->status_blk, sizeof ( struct status_block ) );
	bnx2->status_blk = NULL;
	free_dma ( bnx2->tx_ring.tx_desc_ring, sizeof ( struct bnx2_tx_bd ) );
	bnx2->tx_ring.tx_desc_ring = NULL;
}

static int bnx2_fw_sync ( struct bnx2_nic *bnx2, uint32_t message_data ) {
	int i;
	uint32_t value;

	bnx2->fw_write_sequence++;
	message_data |= bnx2->fw_write_sequence;
	bnx2_shmem_write ( bnx2, BNX2_DRV_MB, message_data );

	/* Wait for acknowledgement */
	for ( i = 0; i < BNX2_FW_ACK_TIMEOUT_MS; i++ ) {
		value = bnx2_shmem_read ( bnx2, BNX2_FW_MB );
		if ( ( value & BNX2_FW_MSG_ACK ) == ( message_data & BNX2_DRV_MSG_SEQ ) )
			break;

		mdelay ( 1 );
	}
	if ( ( value & BNX2_FW_MSG_ACK ) != ( message_data & BNX2_DRV_MSG_SEQ ) &&
		 ( value & BNX2_DRV_MSG_DATA ) != BNX2_DRV_MSG_DATA_WAIT0 ) {
		message_data &= ~BNX2_DRV_MSG_CODE;
		message_data |= BNX2_DRV_MSG_CODE_FW_TIMEOUT;

		bnx2_shmem_write ( bnx2, BNX2_DRV_MB, message_data );
		return -EBUSY;
	}
	if ( ( value & BNX2_FW_MSG_STATUS_MASK ) != BNX2_FW_MSG_STATUS_OK )
		return -EIO;

	return 0;
}

static void bnx2_context_write ( struct bnx2_nic *bnx2,
								  uint32_t cid_addr,
								  uint32_t offset,
								  uint32_t value ) {
	offset += cid_addr;
	if ( CHIP_NUM ( bnx2->misc_id ) == CHIP_NUM_5709 ) {
		int i;
		writel ( value, bnx2->regs + BNX2_CTX_CTX_DATA );
		writel ( offset | BNX2_CTX_CTRL_WRITE_REQ, BNX2_CTX_CTRL );
		for ( i = 0; i < 5; i++ ) {
			value = readl ( bnx2->regs + BNX2_CTX_CTRL );
			if ( ! ( value & BNX2_CTX_CTRL_WRITE_REQ ) )
				break;

			udelay ( 5 );
		}
	} else {
		writel ( offset, bnx2->regs + BNX2_CTX_DATA_ADR );
		writel ( value, bnx2->regs + BNX2_CTX_DATA );
	}
}

static int bnx2_init_context_5709 ( struct bnx2_nic *bnx2 ) {
	/* TODO! */
	( void ) bnx2;
	return 0;
}

static void bnx2_init_context ( struct bnx2_nic *bnx2 ) {
	int i;
	uint32_t vcid;
	uint32_t vcid_addr;
	uint32_t pcid_addr;
	uint32_t offset;

	/* Initialize the 96 Quick CIDs */
	for ( vcid = 0; vcid <= 96; vcid++ ) {
		if ( CHIP_ID ( bnx2->misc_id ) == CHIP_ID_5706_A0 ) {
			uint32_t new_vcid;
			vcid_addr = GET_PHY_CTX_ID_ADDR ( vcid );
			if ( vcid & 0x8 )
				new_vcid = 0x60 + ( vcid & 0xf0 ) + ( vcid & 0x7 );
			else
				new_vcid = vcid;

			pcid_addr = GET_PHY_CTX_ID_ADDR ( new_vcid );
		} else {
			vcid_addr = GET_CTX_ID_ADDR ( vcid );
			pcid_addr = vcid_addr;
		}
		for ( i = 0; i < ( CTX_SIZE / PHY_CTX_SIZE ); i++ ) {
			vcid_addr += ( i << PHY_CTX_SHIFT );
			pcid_addr = vcid_addr;
			writel ( vcid_addr, bnx2->regs + BNX2_CTX_VIRT_ADDR );
			writel ( pcid_addr, bnx2->regs + BNX2_CTX_PAGE_TBL );
			for ( offset = 0; offset < PHY_CTX_SIZE; offset += 4 ) {
				bnx2_context_write ( bnx2, vcid_addr, offset, 0 );
			}
		}

	}
}

static uint32_t rv2p_fw_fixup ( int idx, uint32_t rv2p_code )
{
	switch (idx) {
	case RV2P_P1_FIXUP_PAGE_SIZE_IDX:
		rv2p_code &= ~RV2P_BD_PAGE_SIZE_MASK;
		rv2p_code |= RV2P_BD_PAGE_SIZE;
		break;
	}
	return rv2p_code;
}

static void bnx2_load_rv2p_proc ( struct bnx2_nic *bnx2,
								  const uint32_t *fw,
								  struct bnx2_rv2p_fw_file_entry *entry,
								  int proc ) {
	uint32_t length;
	uint32_t offset;
	uint32_t address;
	uint32_t command;
	uint32_t value;
	uint32_t *code;
	unsigned int i;

	command = ( proc == RV2P_PROC1 ) ?
			  BNX2_RV2P_PROC1_ADDR_CMD_RDWR :
			  BNX2_RV2P_PROC2_ADDR_CMD_RDWR;
	address = ( proc == RV2P_PROC1 ) ?
			  BNX2_RV2P_PROC1_ADDR_CMD :
			  BNX2_RV2P_PROC2_ADDR_CMD;
	length = be32_to_cpu ( entry->rv2p.len );
	offset = be32_to_cpu ( entry->rv2p.offset );
	code = ( uint32_t * ) ( ( ( uint8_t * ) fw ) + offset );
	for ( i = 0; i < length; i += 8 ) {
		writel ( be32_to_cpu ( *code++ ), bnx2->regs + BNX2_RV2P_INSTR_HIGH );
		writel ( be32_to_cpu ( *code++ ), bnx2->regs + BNX2_RV2P_INSTR_LOW );
		value = ( i / 8 ) | command;
		writel ( value, bnx2->regs + address );
	}
	code = ( uint32_t * ) ( ( ( uint8_t * ) fw ) + offset );
	for ( i = 0; i < 8; i++ ) {
		uint32_t location;
		uint32_t c;
		location = be32_to_cpu ( entry->fixup[i] );
		if ( location && ( ( location * 4 ) < length ) ) {
			c = be32_to_cpu ( * ( code + location - 1 ) );
			writel ( c, bnx2->regs + BNX2_RV2P_INSTR_HIGH );
			c = be32_to_cpu ( * ( code + location ) );
			c = rv2p_fw_fixup ( i, c );
			writel ( c, bnx2->regs + BNX2_RV2P_INSTR_LOW );
			value = ( location / 2 ) | command;
			writel ( value, bnx2->regs + address );
		}
	}
	command = ( proc == RV2P_PROC1 ) ?
				BNX2_RV2P_COMMAND_PROC1_RESET :
				BNX2_RV2P_COMMAND_PROC2_RESET;
	writel ( command, bnx2->regs + BNX2_RV2P_COMMAND );
}

static void bnx2_load_rv2p_firmware ( struct bnx2_nic *bnx2,
									  const uint32_t *fw,
									  struct bnx2_rv2p_fw_file *fw_file ) {
	bnx2_load_rv2p_proc ( bnx2, fw, &fw_file->proc1, RV2P_PROC1 );
	bnx2_load_rv2p_proc ( bnx2, fw, &fw_file->proc2, RV2P_PROC2 );
}

static void bnx2_load_mips_firmware_section ( struct bnx2_nic *bnx2,
											  const struct cpu_reg *cpu,
											  const uint32_t *fw,
											  struct bnx2_fw_file_section *section ) {
	uint32_t address, length, file_offset;
	uint32_t offset;
	uint32_t *data;
	address = be32_to_cpu ( section->addr );
	length = be32_to_cpu ( section->len );
	file_offset = be32_to_cpu ( section->offset );
	data = ( uint32_t * ) ( ( ( uint8_t * ) fw ) + file_offset );
	offset = cpu->spad_base + ( address - BNX2_MIPS_VIEW_BASE );
	if ( length ) {
		unsigned int i;
		for ( i = 0; i < length / 4; i++ ) {
			bnx2_reg_write_indirect ( bnx2, offset, be32_to_cpu ( data[i] ) );
			offset += 4;
		}
	}
}

static void bnx2_load_mips_firmware_entry ( struct bnx2_nic *bnx2,
											const struct cpu_reg *cpu,
											const uint32_t *fw,
											struct bnx2_mips_fw_file_entry *entry ) {
	/* We assume the CPU is already halted from the reset */
	uint32_t value;
	value = bnx2_reg_read_indirect ( bnx2, cpu->mode );
	value |= BNX2_CPU_MODE_SOFT_HALT;
	bnx2_reg_write_indirect ( bnx2, cpu->mode, value );
	bnx2_reg_write_indirect ( bnx2, cpu->state, 0xffffff );
	bnx2_load_mips_firmware_section ( bnx2, cpu, fw, &entry->text );
	bnx2_load_mips_firmware_section ( bnx2, cpu, fw, &entry->data );
	bnx2_load_mips_firmware_section ( bnx2, cpu, fw, &entry->rodata );
	/* Clear pre-fetch instruction */
	bnx2_reg_write_indirect ( bnx2, cpu->inst, 0 );
	/* Set the program counter up */
	value = be32_to_cpu ( entry->start_addr );
	bnx2_reg_write_indirect ( bnx2, cpu->pc, value );
	/* Unset halt bit */
	value = bnx2_reg_read_indirect ( bnx2, cpu->mode );
	value &= ~BNX2_CPU_MODE_SOFT_HALT;
	/* Clear CPU states */
	bnx2_reg_write_indirect ( bnx2, cpu->state, 0xffffff );
	bnx2_reg_write_indirect ( bnx2, cpu->mode, value );
}

static void bnx2_load_mips_firmware ( struct bnx2_nic *bnx2,
									  const uint32_t *fw,
									  struct bnx2_mips_fw_file *fw_file ) {
	bnx2_load_mips_firmware_entry ( bnx2, &cpu_reg_rxp, fw, &fw_file->rxp );
	bnx2_load_mips_firmware_entry ( bnx2, &cpu_reg_txp, fw, &fw_file->txp );
	bnx2_load_mips_firmware_entry ( bnx2, &cpu_reg_tpat, fw, &fw_file->tpat );
	bnx2_load_mips_firmware_entry ( bnx2, &cpu_reg_com, fw, &fw_file->com );
	bnx2_load_mips_firmware_entry ( bnx2, &cpu_reg_cp, fw, &fw_file->cp );
}

static void bnx2_load_firmware ( struct bnx2_nic *bnx2 ) {
	const uint32_t *rv2p_firmware;
	const uint32_t *mips_firmware;

	if ( CHIP_NUM ( bnx2->misc_id ) == CHIP_NUM_5709 ) {
		if ( CHIP_ID ( bnx2->misc_id ) == CHIP_ID_5709_A0 ||
			 CHIP_ID ( bnx2->misc_id ) == CHIP_ID_5709_A1 )
			rv2p_firmware = bnx2_rv2p_09ax_firmware;
		else
			rv2p_firmware = bnx2_rv2p_09_firmware;

		mips_firmware = bnx2_mips_09_firmware;
	} else {
		rv2p_firmware = bnx2_rv2p_06_firmware;
		mips_firmware = bnx2_mips_06_firmware;
	}
	bnx2_load_rv2p_firmware ( bnx2, rv2p_firmware,
								   ( struct bnx2_rv2p_fw_file * ) rv2p_firmware );
	bnx2_load_mips_firmware ( bnx2, mips_firmware,
								   ( struct bnx2_mips_fw_file * ) mips_firmware );
}

static int bnx2_reset_chip ( struct bnx2_nic *bnx2 ) {
	uint32_t value;
	int i;
	int rc;

	if ( CHIP_NUM ( bnx2->misc_id ) == CHIP_NUM_5709 ) {
		value = readl ( bnx2->regs + BNX2_MISC_NEW_CORE_CTL );
		value &= ~BNX2_MISC_NEW_CORE_CTL_DMA_ENABLE;
		writel ( value, bnx2->regs + BNX2_MISC_NEW_CORE_CTL );
		value = readl ( bnx2->regs + BNX2_MISC_NEW_CORE_CTL );

		for ( i = 0; i < 100; i++ ) {
			value = readl ( bnx2->regs + BNX2_PCICFG_DEVICE_STATUS );
			if ( ! ( value & BNX2_PCICFG_DEVICE_STATUS_NO_PEND ) )
				break;

			mdelay ( 1 );
		}
	} else {
		value = BNX2_MISC_ENABLE_CLR_BITS_DMA_ENGINE_ENABLE |
				BNX2_MISC_ENABLE_CLR_BITS_TX_DMA_ENABLE |
				BNX2_MISC_ENABLE_CLR_BITS_RX_DMA_ENABLE |
				BNX2_MISC_ENABLE_CLR_BITS_HOST_COALESCE_ENABLE;
		writel ( value, bnx2->regs + BNX2_MISC_ENABLE_CLR_BITS );
		value = readl ( bnx2->regs + BNX2_MISC_ENABLE_CLR_BITS );
		udelay ( 5 );
	}
	if ( ( rc = bnx2_fw_sync ( bnx2, BNX2_DRV_MSG_DATA_WAIT0 |
									 BNX2_DRV_MSG_CODE_RESET ) ) )
		return rc;

	bnx2_shmem_write ( bnx2, BNX2_DRV_RESET_SIGNATURE,
							 BNX2_DRV_RESET_SIGNATURE_MAGIC );
	value = readl ( bnx2->regs + BNX2_MISC_ID );
	if ( CHIP_NUM ( bnx2->misc_id ) == CHIP_NUM_5709 ) {
		writel ( BNX2_MISC_COMMAND_SW_RESET, bnx2->regs + BNX2_MISC_COMMAND );
		readl ( bnx2->regs + BNX2_MISC_COMMAND );
		udelay ( 5 );
		value = BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
				BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP;
		writel ( value, bnx2->regs + BNX2_PCICFG_MISC_CONFIG );
	} else {
		value = BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
				BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
				BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP;
		writel ( value, bnx2->regs + BNX2_PCICFG_MISC_CONFIG );
		mdelay ( 20 );
		value = readl ( bnx2->regs + BNX2_PCICFG_MISC_CONFIG );
		if ( value & ( BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ |
					   BNX2_PCICFG_MISC_CONFIG_CORE_RST_BSY ) )
			return -EBUSY;
	}
	value = readl ( bnx2->regs + BNX2_PCI_SWAP_DIAG0 );
	if ( value != BNX2_PCI_SWAP_DIAG0_VALUE ) {
		DBGC ( bnx2, "BNX2 %p byte swapping setup incorrectly", bnx2 );
		return -ENOTSUP;
	}
	if ( ( rc = bnx2_fw_sync ( bnx2, BNX2_DRV_MSG_DATA_WAIT1 |
									 BNX2_DRV_MSG_CODE_RESET ) ) )
		return rc;

	return 0;
}

static int bnx2_init_chip ( struct bnx2_nic *bnx2 ) {
	uint32_t value;
	int rc;

	value = BNX2_DMA_CONFIG_DATA_BYTE_SWAP |
			BNX2_DMA_CONFIG_DATA_WORD_SWAP |
#if __BYTE_ORDER == __BIG_ENDIAN
			BNX2_DMA_CONFIG_CNTL_BYTE_SWAP |
#endif
			BNX2_DMA_CONFIG_CNTL_WORD_SWAP |
			BNX2_DMA_CONFIG_CNTL_PING_PONG_DMA |
			( 2 << 20 ) |	/* PCI_CLK_CMP_BITS */
			( 1 << 11 ) |	/* CNTL_PCI_COMP_DLY */
			( 1 << 12 ) |	/* DMA read channels */
			( 1 << 16 );	/* DMA write channels */
	writel ( value, bnx2->regs + BNX2_DMA_CONFIG );

	writel ( BNX2_MISC_ENABLE_SET_BITS_CONTEXT_ENABLE |
			 BNX2_MISC_ENABLE_SET_BITS_HOST_COALESCE_ENABLE |
			 BNX2_MISC_ENABLE_SET_BITS_RX_V2P_ENABLE,
			 bnx2->regs + BNX2_MISC_ENABLE_SET_BITS );
	if ( CHIP_NUM ( bnx2->misc_id ) == CHIP_NUM_5709 )
		bnx2_init_context_5709 ( bnx2 );
	else
		bnx2_init_context ( bnx2 );

	bnx2_load_firmware ( bnx2 );
	writel ( BNX2_EMAC_ATTENTION_ENA_LINK, bnx2->regs + BNX2_EMAC_ATTENTION_ENA );
	writel ( ( uint64_t ) virt_to_phys ( bnx2->status_blk ) & 0xffffffff, bnx2->regs + BNX2_HC_STATUS_ADDR_L );
	writel ( ( uint64_t ) virt_to_phys ( bnx2->status_blk ) >> 32, bnx2->regs + BNX2_HC_STATUS_ADDR_H );
	writel ( STATUS_ATTN_BITS_LINK_STATE, bnx2->regs + BNX2_HC_ATTN_BITS_ENABLE );
	if ( ( rc = bnx2_fw_sync ( bnx2, BNX2_DRV_MSG_DATA_WAIT2 | BNX2_DRV_MSG_CODE_RESET ) ) )
		return rc;

	writel ( BNX2_MISC_ENABLE_DEFAULT, bnx2->regs + BNX2_MISC_ENABLE_SET_BITS );
	readl ( bnx2->regs + BNX2_MISC_ENABLE_SET_BITS );
	writel ( BNX2_HC_COMMAND_ENABLE | BNX2_HC_COMMAND_COAL_NOW_WO_INT, bnx2->regs + BNX2_HC_COMMAND );
	udelay ( 20 );
	bnx2->hc_cmd = readl ( bnx2->regs + BNX2_HC_COMMAND );
	return 0;
}

static void bnx2_init_tx_context ( struct bnx2_nic *bnx2 ) {
	uint32_t value;
	uint32_t offset0, offset1, offset2, offset3;

	if ( CHIP_NUM ( bnx2->misc_id ) == CHIP_NUM_5709 ) {
		offset0 = BNX2_L2CTX_TYPE_XI;
		offset1 = BNX2_L2CTX_CMD_TYPE_XI;
		offset2 = BNX2_L2CTX_TBDR_BHADDR_HI_XI;
		offset3 = BNX2_L2CTX_TBDR_BHADDR_LO_XI;
	} else {
		offset0 = BNX2_L2CTX_TYPE;
		offset1 = BNX2_L2CTX_CMD_TYPE;
		offset2 = BNX2_L2CTX_TBDR_BHADDR_HI;
		offset3 = BNX2_L2CTX_TBDR_BHADDR_LO;
	}
	value = BNX2_L2CTX_TYPE_TYPE_L2 | BNX2_L2CTX_TYPE_SIZE_L2;
	bnx2_context_write ( bnx2, GET_CTX_ID_ADDR ( TX_CID ), offset0, value );

	value = BNX2_L2CTX_CMD_TYPE_TYPE_L2 | ( 8 << 16 );
	bnx2_context_write ( bnx2, GET_CTX_ID_ADDR ( TX_CID ), offset1, value );

	value = ( uint64_t ) virt_to_bus ( bnx2->tx_ring.tx_desc_ring ) >> 32;
	bnx2_context_write ( bnx2, GET_CTX_ID_ADDR ( TX_CID ), offset2, value );

	value = ( uint64_t ) virt_to_bus ( bnx2->tx_ring.tx_desc_ring ) & 0xffffffff;
	bnx2_context_write ( bnx2, GET_CTX_ID_ADDR ( TX_CID ), offset3, value );
}

static void bnx2_init_rings ( struct bnx2_nic *bnx2 ) {
	struct bnx2_tx_bd *txbd = bnx2->tx_ring.tx_desc_ring;

	txbd->haddr = virt_to_bus ( bnx2->tx_ring.tx_desc_ring );
	bnx2->tx_ring.tx_prod = 0;
	bnx2->tx_ring.tx_prod_bseq = 0;
	bnx2_init_tx_context ( bnx2 );
}

static int bnx2_reset_nic ( struct bnx2_nic *bnx2 ) {
	int rc;
	if ( ( rc = bnx2_reset_chip ( bnx2 ) ) != 0 )
		return rc;

	if ( ( rc = bnx2_init_chip ( bnx2 ) ) != 0 )
		return rc;

	bnx2_init_rings ( bnx2 );
	return 0;
}

static int bnx2_init_nic ( struct bnx2_nic *bnx2 ) {
	int rc;
	uint32_t value;

	if ( ( rc = bnx2_reset_nic ( bnx2 ) ) != 0 )
		return rc;

	value = readl ( bnx2->regs + BNX2_EMAC_MODE );
	value |= BNX2_EMAC_MODE_PORT_GMII;
	writel ( value, bnx2->regs + BNX2_EMAC_MODE );
	return 0;
}

/******************************************************************************
 *
 * MII interface
 *
 ******************************************************************************
 */

/**
 * Read from MII register
 *
 * @v mii		MII interface
 * @v reg		Register address
 * @ret value		Data read, or negative error
 */
static int bnx2_mii_read ( struct mii_interface *mii, unsigned int reg ) {
	struct bnx2_nic *bnx2 =
		container_of ( mii, struct bnx2_nic, mii );
	uint32_t value;
	int i;

	/* Check to see if there's a pending transaction */
	value = readl ( bnx2->regs + BNX2_EMAC_MDIO_COMM );
	if ( value & BNX2_EMAC_MDIO_COMM_START_BUSY )
		return -EBUSY;

	value = ( bnx2->phy_addr << 21 ) | ( reg << 16 ) |
			BNX2_EMAC_MDIO_COMM_COMMAND_READ |
			BNX2_EMAC_MDIO_COMM_START_BUSY;
	writel ( value, bnx2->regs + BNX2_EMAC_MDIO_COMM );
	for ( i = 0; i < 50; i++ ) {
		value = readl ( bnx2->regs + BNX2_EMAC_MDIO_COMM );
		if ( ! ( value & BNX2_EMAC_MDIO_COMM_START_BUSY ) ) {
			udelay ( 5 );
			value = readl ( bnx2->regs + BNX2_EMAC_MDIO_COMM );
			break;
		}
		udelay ( 10 );
	}
	if ( value & BNX2_EMAC_MDIO_COMM_START_BUSY )
		value = -EBUSY;
	else if ( value & BNX2_EMAC_MDIO_COMM_FAIL )
		value = -EIO;

	return value;
}

/**
 * Write to MII register
 *
 * @v mii		MII interface
 * @v reg		Register address
 * @v data		Data to write
 * @ret rc		Return status code
 */
static int bnx2_mii_write ( struct mii_interface *mii, unsigned int reg,
				unsigned int data) {
	struct bnx2_nic *bnx2 =
		container_of ( mii, struct bnx2_nic, mii );
	uint32_t value;
	int i;
	int rc;

	/* Check to see if there's a pending transaction */
	value = readl ( bnx2->regs + BNX2_EMAC_MDIO_COMM );
	if ( value & BNX2_EMAC_MDIO_COMM_START_BUSY )
		return -EBUSY;

	value = ( bnx2->phy_addr << 21 ) | ( reg << 16 ) | data |
			BNX2_EMAC_MDIO_COMM_COMMAND_WRITE |
			BNX2_EMAC_MDIO_COMM_START_BUSY;
	writel ( value, bnx2->regs + BNX2_EMAC_MDIO_COMM );

	for ( i = 0; i < 50; i++ ) {
		value = readl ( bnx2->regs + BNX2_EMAC_MDIO_COMM );
		if ( ! ( value & BNX2_EMAC_MDIO_COMM_START_BUSY ) ) {
				udelay ( 5 );
				break;
		}
		udelay ( 10 );
	}
	if ( value & BNX2_EMAC_MDIO_COMM_START_BUSY )
		rc = -EBUSY;
	else if ( value & BNX2_EMAC_MDIO_COMM_FAIL )
		rc = -EIO;
	else
		rc = 0;

	return rc;
}

/** bnx2 MII operations */
static struct mii_operations bnx2_mii_operations = {
	.read = bnx2_mii_read,
	.write = bnx2_mii_write,
};

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Reset hardware
 *
 * @v bnx2		bnx2 device
 * @ret rc		Return status code
 */
static int bnx2_reset ( struct bnx2_nic *bnx2 ) {
	uint32_t value;

	writel ( BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA |
			 BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP,
			 bnx2->regs + BNX2_PCICFG_MISC_CONFIG );

	bnx2->misc_id = readl ( bnx2->regs + BNX2_MISC_ID );
	bnx2_shmem_init ( bnx2 );
	value = bnx2_shmem_read ( bnx2, BNX2_DEV_INFO_SIGNATURE );
	if ( ( value & BNX2_DEV_INFO_SIGNATURE_MAGIC_MASK ) !=
		 BNX2_DEV_INFO_SIGNATURE_MAGIC ) {
		DBGC ( bnx2, "BNX2 %p firmware is not running\n", bnx2 );
		return -EIO;
	}
	bnx2->phy_addr = 1;
	return 0;
}

/******************************************************************************
 *
 * Link state
 *
 ******************************************************************************
 */

/**
 * Check link state
 *
 * @v netdev		Network device
 */
static void bnx2_check_link ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;
	struct status_block *status_blk = bnx2->status_blk;
	uint32_t status_idx = status_blk->status_idx;
	uint32_t new_link = bnx2->status_blk->status_attn_bits & STATUS_ATTN_BITS_LINK_STATE;
	uint32_t old_link = bnx2->status_blk->status_attn_bits_ack & STATUS_ATTN_BITS_LINK_STATE;

	if ( new_link != old_link ) {
		if ( new_link ) {
			writel ( STATUS_ATTN_BITS_LINK_STATE, bnx2->regs + BNX2_PCICFG_STATUS_BIT_SET_CMD );
			netdev_link_up ( netdev );
		}
		else {
			writel ( STATUS_ATTN_BITS_LINK_STATE, bnx2->regs + BNX2_PCICFG_STATUS_BIT_CLEAR_CMD );
			netdev_link_down ( netdev );
		}
		//writel ( BNX2_PCICFG_INT_ACK_CMD_MASK_INT | BNX2_PCICFG_INT_ACK_CMD_INDEX_VALID | status_idx, bnx2->regs + BNX2_PCICFG_INT_ACK_CMD );
		//writel ( BNX2_PCICFG_INT_ACK_CMD_INDEX_VALID | status_idx, bnx2->regs + BNX2_PCICFG_INT_ACK_CMD );
		writel ( bnx2->hc_cmd | BNX2_HC_COMMAND_COAL_NOW_WO_INT, bnx2->regs + BNX2_HC_COMMAND );
		readl ( bnx2->regs + BNX2_HC_COMMAND );
	}
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int bnx2_open ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;
	int rc;

	if ( ( rc = bnx2_init_nic ( bnx2 ) ) )
		return rc;

	bnx2_check_link ( netdev );
	return 0;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void bnx2_close ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;
	( void ) bnx2;
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int bnx2_transmit ( struct net_device *netdev,
			       struct io_buffer *iobuf ) {
	struct bnx2_nic *bnx2 = netdev->priv;
	struct bnx2_tx_ring_info *txr = &bnx2->tx_ring;
	struct bnx2_tx_bd *txbd;
	unsigned int tx_idx;
	unsigned int tx_tail;

	if ( ( txr->tx_prod - txr->tx_cons ) >= TX_DESC_CNT ) {
		DBGC ( bnx2, "BNX2 %p out of transmit descriptors\n", bnx2 );
		return -ENOBUFS;
	}
	tx_idx = ( txr->tx_prod++ % TX_DESC_CNT );
	tx_tail = ( txr->tx_prod % TX_DESC_CNT );
	txbd = &txr->tx_desc_ring[tx_idx];
	txbd->haddr = ( ( uint64_t ) virt_to_bus ( iobuf->data ) ) << 32;
	txbd->reserved = iob_len ( iobuf );
	txbd->vlan_tag = TX_BD_FLAGS_START | TX_BD_FLAGS_END;
	wmb();
	txr->tx_prod_bseq += iob_len ( iobuf );
	writew ( tx_tail, bnx2->regs + MB_TX_CID_ADDR + BNX2_L2CTX_TX_HOST_BIDX );
	writel ( txr->tx_prod_bseq, bnx2->regs + MB_TX_CID_ADDR + BNX2_L2CTX_TX_HOST_BSEQ );
	return 0;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void bnx2_poll ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;
	( void ) bnx2;
	bnx2_check_link ( netdev );
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void bnx2_irq ( struct net_device *netdev, int enable ) {
	struct bnx2_nic *bnx2 = netdev->priv;

	DBGC ( bnx2, "BNX2 %p does not yet support interrupts\n", bnx2 );
	( void ) enable;
}

/** bnx2 network device operations */
static struct net_device_operations bnx2_operations = {
	.open		= bnx2_open,
	.close		= bnx2_close,
	.transmit	= bnx2_transmit,
	.poll		= bnx2_poll,
	.irq		= bnx2_irq,
};

/******************************************************************************
 *
 * PCI interface
 *
 ******************************************************************************
 */

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @ret rc		Return status code
 */
static int bnx2_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct bnx2_nic *bnx2;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *bnx2 ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &bnx2_operations );
	bnx2 = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( bnx2, 0, sizeof ( *bnx2 ) );
	bnx2->netdev = netdev;
	/* Fix up PCI device */
	adjust_pci_device ( pci );

	bnx2->pci = pci;

	/* Map registers */
	bnx2->regs = ioremap ( pci->membase, BNX2_BAR_SIZE );
	if ( ( rc = bnx2_alloc_mem ( bnx2 ) ) )
		goto err_alloc;

	/* Reset the NIC */
	if ( ( rc = bnx2_reset ( bnx2 ) ) != 0 )
		goto err_reset;

	bnx2_set_mac_address ( bnx2, netdev );
	/* Initialise and reset MII interface */
	mii_init ( &bnx2->mii, &bnx2_mii_operations );
	/*
	if ( ( rc = mii_reset ( &bnx2->mii ) ) != 0 ) {
		DBGC ( bnx2, "BNX2 %p could not reset MII: %s\n",
		       bnx2, strerror ( rc ) );
		goto err_mii_reset;
	}
	*/
	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	bnx2_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
/*
 err_mii_reset:
	bnx2_reset ( bnx2 );
*/
 err_reset:
	netdev_nullify ( netdev );
	netdev_put ( netdev );
 err_alloc:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void bnx2_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct bnx2_nic *bnx2 = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset card */
	bnx2_reset ( bnx2 );

	bnx2_free_mem ( bnx2 );

	/* Free network device */
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** bnx2 PCI device IDs */
static struct pci_device_id bnx2_nics[] = {
	PCI_ROM ( 0x14e4, 0x164c, "bnx2-5708C", "Broadcom NetXtreme II BCM5708C", 0 ),
};

/** bnx2 PCI driver */
struct pci_driver bnx2_driver __pci_driver = {
	.ids = bnx2_nics,
	.id_count = ( sizeof ( bnx2_nics ) / sizeof ( bnx2_nics[0] ) ),
	.probe = bnx2_probe,
	.remove = bnx2_remove,
};
