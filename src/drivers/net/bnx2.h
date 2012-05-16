#ifndef _BNX2_H
#define _BNX2_H

/** @file
 *
 * Broadcom NetXtreme II series 10/100/1000 network card driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <ipxe/mii.h>

/** bnx2 BAR size */
#define BNX2_BAR_SIZE 256

/** MISC_ID register */
#define BNX2_MISC_ID			0x00000808UL
#define BNX2_MISC_ID_BOND_ID	0x0000000fUL
#define BNX2_MISC_ID_CHIP_METAL	0x00000ff0UL
#define BNX2_MISC_ID_CHIP_REV	0x0000f000UL
#define BNX2_MISC_ID_CHIP_NUM	0xffff0000UL

/** PCICFG_MISC_CONFIG register */
#define BNX2_PCICFG_MISC_CONFIG                         0x00000068UL
#define BNX2_PCICFG_MISC_CONFIG_TARGET_MB_WORD_SWAP     0x00000008UL
#define BNX2_PCICFG_MISC_CONFIG_REG_WINDOW_ENA          0x00000080UL
#define BNX2_PCICFG_MISC_CONFIG_CORE_RST_REQ            0x00000100UL
#define BNX2_PCICFG_MISC_CONFIG_CORE_RST_BSY            0x00000200UL

/** EMAC_MDIO_COMM register */
#define BNX2_EMAC_MDIO_COMM                     0x000014acUL
#define BNX2_EMAC_MDIO_COMM_COMMAND_WRITE       0x04000000UL
#define BNX2_EMAC_MDIO_COMM_COMMAND_READ        0x08000000UL
#define BNX2_EMAC_MDIO_COMM_START_BUSY          0x20000000UL
#define BNX2_EMAC_MDIO_COMM_FAIL                0x10000000UL

/** A bnx2 network card */
struct bnx2_nic {
	/** Registers */
	void *regs;
	/** PHY address */
	uint32_t phy_addr;
	/** MII interface */
	struct mii_interface mii;
};

#endif /* _BNX2_H */
