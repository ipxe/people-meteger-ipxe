/*
 * Copyright (C) 2012 Adrian Jamroz <adrian.jamroz@gmail.com>
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
#include "rhine.h"

#define	rhine_setbit(_reg, _mask)	writeb ( readb ( _reg ) | _mask, _reg )

#define virt_to_le32bus(_adr)	cpu_to_le32 ( virt_to_bus ( _adr ) )

/** @file
 *
 * VIA Rhine network driver
 *
 */

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
static int rhine_mii_read ( struct mii_interface *mii, unsigned int reg ) {
	struct rhine_nic *rhn = container_of ( mii, struct rhine_nic, mii );
	int timeout = RHINE_TIMEOUT_US;

	DBGC2 ( rhn, "RHINE %p MII read reg %d\n", rhn, reg );

	writeb ( reg, rhn->regs + RHINE_MII_ADDR );
	rhine_setbit( rhn->regs + RHINE_MII_CR, RHINE_MII_CR_RDEN );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( rhn->regs + RHINE_MII_CR )
		          & RHINE_MII_CR_RDEN ) == 0 )
			return readw ( rhn->regs + RHINE_MII_RDWR );
	}

	DBGC ( rhn, "MII read timeout\n" );
	return -ETIMEDOUT;
}

/**
 * Write to MII register
 *
 * @v mii		MII interface
 * @v reg		Register address
 * @v data		Data to write
 * @ret rc		Return status code
 */
static int rhine_mii_write ( struct mii_interface *mii, unsigned int reg,
                             unsigned int data ) {
	struct rhine_nic *rhn = container_of ( mii, struct rhine_nic, mii );
	int timeout = RHINE_TIMEOUT_US;

	DBGC2 ( rhn, "RHINE %p MII write reg %d data 0x%04x\n",
	        rhn, reg, data );

	writeb ( reg, rhn->regs + RHINE_MII_ADDR );
	writew ( data, rhn->regs + RHINE_MII_RDWR );
	rhine_setbit ( rhn->regs + RHINE_MII_CR, RHINE_MII_CR_WREN );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( rhn->regs + RHINE_MII_CR )
		           & RHINE_MII_CR_WREN ) == 0 )
			return 0;
	}

	DBGC ( rhn, "MII write timeout\n" );
	return -ETIMEDOUT;
}

/** Rhine MII operations */
static struct mii_operations rhine_mii_operations = {
	.read = rhine_mii_read,
	.write = rhine_mii_write,
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
 * We're using PIO because this might reset the MMIO enable bit.
 *
 * @v rhn		Rhine device
 * @v ioaddr		PIO address
 * @ret rc		Return status code
 */
static int rhine_reset ( struct rhine_nic *rhn, unsigned long ioaddr ) {
	int timeout = RHINE_TIMEOUT_US;
	uint8_t cr1;

	DBGC ( rhn, "RHINE %p reset\n", rhn );

	cr1 = inb ( ioaddr + RHINE_CR1 );
	outb ( cr1 | RHINE_CR1_RESET, ioaddr + RHINE_CR1 );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( inb ( ioaddr + RHINE_CR1 ) & RHINE_CR1_RESET ) == 0 )
			return 0;
	}

	DBGC ( rhn, "RHINE %p reset timeout\n", rhn );
	return -ETIMEDOUT;
}

/**
 * Enable MMIO register access
 *
 * @v ioaddr		PIO address
 * @v revision		Card revision
 */
static void rhine_enable_mmio ( unsigned long ioaddr, int revision ) {
	uint8_t conf;

	if ( revision < RHINE_REVISION_OLD ) {
		conf = inb ( ioaddr + RHINE_CHIPCFG_A );
		outb ( conf | RHINE_CHIPCFG_A_MMIO, ioaddr + RHINE_CHIPCFG_A );
	} else {
		conf = inb ( ioaddr + RHINE_CHIPCFG_D );
		outb ( conf | RHINE_CHIPCFG_D_MMIO, ioaddr + RHINE_CHIPCFG_D );
	}
}

/**
 * Reload EEPROM contents
 *
 * We're using PIO because this might reset the MMIO enable bit.
 *
 * @v rhn		Rhine device
 * @v ioaddr		PIO address
 */
static int rhine_reload_eeprom ( struct rhine_nic *rhn, unsigned long ioaddr ) {
	int timeout = RHINE_TIMEOUT_US;
	uint8_t eeprom;

	eeprom = inb ( ioaddr + RHINE_EEPROM_CTRL );

	/* set EEPROM Reload bit */
	outb ( eeprom | RHINE_EEPROM_CTRL_RELOAD, ioaddr + RHINE_EEPROM_CTRL );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( inb ( ioaddr + RHINE_EEPROM_CTRL )
		           & RHINE_EEPROM_CTRL_RELOAD ) == 0 )
			return 0;
	}

	DBGC ( rhn, "RHINE %p EEPROM reload timeout\n", rhn );
	return -ETIMEDOUT;
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
static void rhine_check_link ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;

	if ( readb ( rhn->regs + RHINE_MII_SR ) & RHINE_MII_SR_LINKPOLL )
		netdev_link_down ( netdev );
	else
		netdev_link_up ( netdev );

	if ( readb ( rhn->regs + RHINE_MII_SR ) & RHINE_MII_SR_PHYERR )
		netdev_link_err ( netdev, -EINVAL );
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

static int rhine_alloc_rings ( struct rhine_nic *rhn )
{
	int i;

	/* Allocate RX descriptor ring */
	rhn->rx_prod = 0;
	rhn->rx_cons = 0;
	rhn->rx_ring = malloc_dma ( RHINE_RXDESC_SIZE, RHINE_RING_ALIGN );
	if ( ! rhn->rx_ring ) {
		DBGC ( rhn, "RHINE %p failed to allocate RX ring\n", rhn );
		return -ENOMEM;
	}

	memset ( rhn->rx_ring, 0, RHINE_RXDESC_SIZE );

	for ( i = 0; i < RHINE_RXDESC_NUM; i++ )
		rhn->rx_ring[i].next = virt_to_le32bus (
		    &rhn->rx_ring[(i + 1) % RHINE_RXDESC_NUM] );

	DBGC2 ( rhn, "RHINE %p RX ring start address: %p\n", rhn, rhn->rx_ring );

	/* Allocate TX descriptor ring */
	rhn->tx_prod = 0;
	rhn->tx_cons = 0;
	rhn->tx_ring = malloc_dma ( RHINE_TXDESC_SIZE, RHINE_RING_ALIGN );
	if ( ! rhn->tx_ring ) {
		DBGC ( rhn, "RHINE %p failed to allocate TX ring\n", rhn );
		goto err_tx_ring_alloc;
	}

	memset ( rhn->tx_ring, 0, RHINE_TXDESC_SIZE );

	for ( i = 0; i < RHINE_TXDESC_NUM; i++ )
		rhn->tx_ring[i].next = virt_to_le32bus (
		    &rhn->tx_ring[(i + 1) % RHINE_TXDESC_NUM] );

	DBGC2 ( rhn, "RHINE %p TX ring start address: %p\n", rhn, rhn->tx_ring );

	/* Program TX and RX ring start address */
	writel ( virt_to_le32bus ( rhn->tx_ring ),
	         rhn->regs + RHINE_TXQUEUE_BASE );
	writel ( virt_to_le32bus ( rhn->rx_ring ),
	         rhn->regs + RHINE_RXQUEUE_BASE );

	return 0;

err_tx_ring_alloc:
	free_dma ( rhn->rx_ring, RHINE_RXDESC_SIZE );
	return -ENOMEM;
}

static void rhine_refill_rx ( struct rhine_nic *rhn )
{
	struct rhine_descriptor *desc;
	struct io_buffer *iobuf;
	int rx_idx, i = 0;

	while ( ( rhn->rx_prod - rhn->rx_cons ) < RHINE_RXDESC_NUM ) {
		iobuf = alloc_iob ( RHINE_RX_MAX_LEN );
		if ( ! iobuf ) {
			DBGC2 ( rhn, "alloc_iob() failed\n" );
			return;
		}

		rx_idx = ( rhn->rx_prod++ % RHINE_RXDESC_NUM );
		desc = &rhn->rx_ring[rx_idx];
		desc->buffer = virt_to_le32bus ( iobuf->data );
		desc->des1 = cpu_to_le32 ( RHINE_DES1_SIZE (
		    RHINE_RX_MAX_LEN - 1 ) | RHINE_DES1_CHAIN |
		    RHINE_DES1_IC );

		wmb();
		desc->des0 = cpu_to_le32 ( RHINE_DES0_OWN );

		rhn->rx_buffs[rx_idx] = iobuf;
		i++;
	}

	DBGC2 ( rhn, "RHINE %p refilled %d RX descriptors\n", rhn, i );
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int rhine_open ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;
	int timeout = RHINE_TIMEOUT_US;
	int rc;

	DBGC2 ( rhn, "RHINE %p regs at: %p\n", rhn, rhn->regs );

	if ( ( rc = rhine_alloc_rings ( rhn ) ) != 0 )
		return rc;

	writeb ( RHINE_RCR_PHYS_ACCEPT | RHINE_RCR_BCAST_ACCEPT |
	    RHINE_RCR_RUNT_ACCEPT, rhn->regs + RHINE_RCR );

	/* Enable link status monitoring */
	writeb ( MII_BMSR, rhn->regs + RHINE_MII_ADDR );
	writeb ( RHINE_MII_CR_AUTOPOLL, rhn->regs + RHINE_MII_CR );

	/* Wait for MDIO auto-poll to complete */
	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( rhn->regs + RHINE_MII_ADDR )
		               & RHINE_MII_ADDR_MDONE ) == 0 )
			break;
	}

	if ( timeout == 0 ) {
		DBGC ( rhn, "RHINE %p MDIO auto-poll timeout\n", rhn );
		return -ETIMEDOUT;
	}

	writeb ( MII_BMSR | RHINE_MII_ADDR_MSRCEN, rhn->regs + RHINE_MII_ADDR );

	/* Some cards need an extra delay(observed with VT6102) */
	mdelay ( 10 );

	/* Enable interrupts */
	writeb ( 0xff, rhn->regs + RHINE_IMR0 );
	writeb ( 0xff, rhn->regs + RHINE_IMR1 );

	/* Enable RX/TX of packets */
	writeb ( RHINE_CR0_STARTNIC | RHINE_CR0_RXEN | RHINE_CR0_TXEN,
	    rhn->regs + RHINE_CR0 );

	/* Enable Full duplex */
	writeb ( RHINE_CR1_FDX, rhn->regs + RHINE_CR1 );

	rhine_refill_rx ( rhn );
	rhine_check_link ( netdev );

	return 0;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void rhine_close ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;
	int i;

	DBGC ( rhn, "RHINE %p close\n", rhn );

	/* Stop card, clear RXON and TXON bits */
	writeb ( RHINE_CR0_STOPNIC, rhn->regs + RHINE_CR0 );

	/* Clear RX ring address */
	writel ( 0, rhn->regs + RHINE_RXQUEUE_BASE );

	/* Destroy RX ring */
	free_dma ( rhn->rx_ring, RHINE_RXDESC_SIZE );
	rhn->rx_ring = NULL;
	rhn->rx_prod = 0;
	rhn->rx_cons = 0;

	/* Discard receive buffers */
	for ( i = 0 ; i < RHINE_RXDESC_NUM ; i++ ) {
		if ( rhn->rx_buffs[i] )
			free_iob ( rhn->rx_buffs[i] );
		rhn->rx_buffs[i] = NULL;
	}

	/* Clear TX ring address */
	writel ( 0, rhn->regs + RHINE_TXQUEUE_BASE );

	/* Destroy TX ring */
	free_dma ( rhn->tx_ring, RHINE_TXDESC_SIZE );
	rhn->tx_ring = NULL;
	rhn->tx_prod = 0;
	rhn->tx_cons = 0;

	/* Disable interrupts */
	writeb ( 0, RHINE_IMR0 );
	writeb ( 0, RHINE_IMR1 );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int rhine_transmit ( struct net_device *netdev,
                            struct io_buffer *iobuf ) {
	struct rhine_nic *rhn = netdev->priv;
	struct rhine_descriptor *desc;
	int tx_idx;

	if ( ( rhn->tx_prod - rhn->tx_cons ) >= RHINE_TXDESC_NUM ) {
		DBGC ( rhn, "RHINE %p out of TX buffers\n", rhn );
		return -ENOBUFS;
	}

	iob_pad ( iobuf, ETH_ZLEN );

	tx_idx = ( rhn->tx_prod++ % RHINE_TXDESC_NUM );
	desc = &rhn->tx_ring[tx_idx];
	desc->buffer = virt_to_le32bus ( iobuf->data );
	desc->des1 = cpu_to_le32 ( RHINE_DES1_IC | RHINE_TDES1_STP
	    | RHINE_TDES1_EDP | RHINE_DES1_CHAIN
	    | RHINE_DES1_SIZE ( iob_len ( iobuf ) ) );

	DBGC2 ( rhn, "RHINE %p tx_prod=%d desc=%p iobuf=%p len=%d\n",
	        rhn, tx_idx, desc, iobuf->data, iob_len ( iobuf ) );

	/* Card will auto-poll descriptor. Hence wmb() before setting OWN bit */
	wmb();
	desc->des0 = cpu_to_le32 ( RHINE_DES0_OWN );
	rhine_setbit ( rhn->regs + RHINE_CR1, RHINE_CR1_TXPOLL );
	return 0;
}

static void rhine_poll_rx ( struct rhine_nic *rhn ) {
	struct rhine_descriptor *desc;
	struct io_buffer *iobuf;
	int rx_idx;

	while ( rhn->rx_cons != rhn->rx_prod ) {
		rx_idx = ( rhn->rx_cons % RHINE_RXDESC_NUM );
		desc = &rhn->rx_ring[rx_idx];

		if ( le32_to_cpu ( desc->des0 ) & RHINE_DES0_OWN )
			return;

		DBGC2 ( rhn, "RHINE %p got packet on idx=%d (prod=%d)\n",
		    rhn, rx_idx, rhn->rx_prod % RHINE_RXDESC_NUM);

		iobuf = rhn->rx_buffs[rx_idx];

		/* Set packet length, strip CRC field */
		iob_put ( iobuf, RHINE_DES0_GETSIZE (
		          le32_to_cpu ( desc->des0 ) ) - 4 );

		if ( ! ( le32_to_cpu ( desc->des0 ) & RHINE_RDES0_RXOK ) )
			netdev_rx_err ( rhn->netdev, iobuf, -EIO);
		else
			netdev_rx ( rhn->netdev, iobuf );

		rhn->rx_cons++;
	}
}

static void rhine_poll_tx ( struct rhine_nic *rhn ) {
	struct rhine_descriptor *desc;
	int tx_idx;

	while ( rhn->tx_cons != rhn->tx_prod ) {
		tx_idx = ( rhn->tx_cons % RHINE_TXDESC_NUM );
		desc = &rhn->tx_ring[tx_idx];

		if ( le32_to_cpu ( desc->des0 ) & RHINE_DES0_OWN )
			return;

		netdev_tx_complete_next ( rhn->netdev );

		DBGC2 ( rhn, "RHINE %p poll_tx cons=%d prod=%d tsr=%04x\n",
		    rhn, tx_idx, rhn->tx_prod % RHINE_TXDESC_NUM, desc->des0 );
		rhn->tx_cons++;
	}

}
/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void rhine_poll ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;
	uint8_t isr0, isr1;

	isr0 = readb ( rhn->regs + RHINE_ISR0 );
	isr1 = readb ( rhn->regs + RHINE_ISR1 );

	/* ACK interrupts */
	writeb ( isr0, rhn->regs + RHINE_ISR0 );
	writeb ( isr1, rhn->regs + RHINE_ISR1 );

	if ( isr0 & ( RHINE_ISR0_TXDONE | RHINE_ISR0_TXERR ) )
		rhine_poll_tx ( rhn );

	if ( isr0 & ( RHINE_ISR0_RXDONE | RHINE_ISR0_RXERR ) )
		rhine_poll_rx ( rhn );

	/* Once RX underrun occurrs the RXDONE indications seem to get lost */
	if ( isr1 & RHINE_ISR1_RXNOBUF )
		rhine_poll_rx ( rhn );

	if ( isr0 & RHINE_ISR0_RXRINGERR )
		DBGC ( rhn, "RHINE %p RXRINGERR interrupt\n", rhn );

	if ( isr1 & RHINE_ISR1_PORTSTATE )
		rhine_check_link ( netdev );

	rhine_refill_rx ( rhn );
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void rhine_irq ( struct net_device *netdev, int enable ) {
	struct rhine_nic *nic = netdev->priv;

	if (enable) {
		/* Enable interrupts */
		writeb ( 0xff, nic->regs + RHINE_IMR0 );
		writeb ( 0xff, nic->regs + RHINE_IMR1 );
	} else {
		/* Disable interrupts */
		writeb ( 0, nic->regs + RHINE_IMR0 );
		writeb ( 0, nic->regs + RHINE_IMR1 );
	}
}

/** Rhine network device operations */
static struct net_device_operations rhine_operations = {
	.open		= rhine_open,
	.close		= rhine_close,
	.transmit	= rhine_transmit,
	.poll		= rhine_poll,
	.irq		= rhine_irq,
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
static int rhine_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct rhine_nic *rhn;
	int rc;
	uint8_t revision;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *rhn ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	netdev_init ( netdev, &rhine_operations );
	rhn = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( rhn, 0, sizeof ( *rhn ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Read card revision */
	pci_read_config_byte ( pci, PCI_REVISION, &revision );
	DBGC2 ( rhn, "RHINE %p Revision %#02x detected\n", rhn, revision );

	/* Map registers */
	rhn->netdev = netdev;
	rhn->regs = ioremap ( pci->membase, RHINE_BAR_SIZE );

	/* Reset the NIC */
	if ( ( rc = rhine_reset ( rhn, pci->ioaddr ) ) != 0 ) {
		DBGC ( rhn, "RHINE %p reset failed\n", rhn );
		goto err_reset;
	}

	/* Reload EEPROM */
	if ( ( rc = rhine_reload_eeprom ( rhn, pci->ioaddr ) ) != 0 ) {
		DBGC ( rhn, "RHINE %p EEPROM reload failed\n", rhn );
		goto err_reset;
	}

	/* Enable MMIO */
	rhine_enable_mmio ( pci->ioaddr, revision );

	netdev->hw_addr[0] = readb ( rhn->regs + RHINE_MAC0 );
	netdev->hw_addr[1] = readb ( rhn->regs + RHINE_MAC1 );
	netdev->hw_addr[2] = readb ( rhn->regs + RHINE_MAC2 );
	netdev->hw_addr[3] = readb ( rhn->regs + RHINE_MAC3 );
	netdev->hw_addr[4] = readb ( rhn->regs + RHINE_MAC4 );
	netdev->hw_addr[5] = readb ( rhn->regs + RHINE_MAC5 );

	/* Initialise and reset MII interface */
	mii_init ( &rhn->mii, &rhine_mii_operations );
	if ( ( rc = mii_reset ( &rhn->mii ) ) != 0 ) {
		DBGC ( rhn, "RHINE %p could not reset MII: %s\n",
		       rhn, strerror ( rc ) );
		goto err_mii_reset;
	}

	DBGC2 ( rhn, "RHINE PHY vendor id: %04x\n",
	        rhine_mii_read ( &rhn->mii, 0x02 ) );
	DBGC2 ( rhn, "RHINE PHY device id: %04x\n",
	       rhine_mii_read ( &rhn->mii, 0x03 ) );

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	rhine_check_link ( netdev );

	return 0;

 err_register_netdev:
 err_mii_reset:
	rhine_reset ( rhn, pci->ioaddr );
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
static void rhine_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct rhine_nic *nic = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset card */
	rhine_reset ( nic, pci->ioaddr );

	/* Free network device */
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** Rhine PCI device IDs */
static struct pci_device_id rhine_nics[] = {
	PCI_ROM ( 0x1106, 0x3065, "dlink-530tx", "VIA VT6102", 0 ),
	PCI_ROM ( 0x1106, 0x3106, "vt6105", "VIA VT6105", 0 ),
	PCI_ROM ( 0x1106, 0x3043, "dlink-530tx-old", "VIA VT3043", 0 ),
	PCI_ROM ( 0x1106, 0x3053, "vt6105m", "VIA VT6105M", 0 ),
	PCI_ROM ( 0x1106, 0x6100, "via-rhine-old", "VIA 86C100A", 0 )
};

/** Rhine PCI driver */
struct pci_driver rhine_driver __pci_driver = {
	.ids = rhine_nics,
	.id_count = ( sizeof ( rhine_nics ) / sizeof ( rhine_nics[0] ) ),
	.probe = rhine_probe,
	.remove = rhine_remove,
};
