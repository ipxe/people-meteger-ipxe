/*
 * Copyright (C) 2012 Michael Brown <mbrown@fensystems.co.uk>.
 * Copyright (C) 2013 Thomas Miletich <thomas.miletich@gmail.com>
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
#include "icp1k.h"

/** @file
 *
 * IC+ 1000 network driver
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
static int icp1k_mii_read ( struct mii_interface *mii, unsigned int reg ) {
	struct icp1k_nic *icp =
		container_of ( mii, struct icp1k_nic, mii );
return 0;
	DBGC ( icp, "SKELETON %p does not yet support MII read\n", icp );
	( void ) reg;
	return -ENOTSUP;
}

/**
 * Write to MII register
 *
 * @v mii		MII interface
 * @v reg		Register address
 * @v data		Data to write
 * @ret rc		Return status code
 */
static int icp1k_mii_write ( struct mii_interface *mii, unsigned int reg,
				unsigned int data) {
	struct icp1k_nic *icp =
		container_of ( mii, struct icp1k_nic, mii );
return 0;
	DBGC ( icp, "SKELETON %p does not yet support MII write\n", icp );
	( void ) reg;
	( void ) data;
	return -ENOTSUP;
}

/** IC+ 1000 MII operations */
static struct mii_operations icp1k_mii_operations = {
	.read = icp1k_mii_read,
	.write = icp1k_mii_write,
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
 * @v icp		IC+ 1000 device
 * @ret rc		Return status code
 */
static int icp1k_reset ( struct icp1k_nic *icp ) {
	uint32_t reset;
	int timeout = ICP1K_TIMEOUT_US;

	/* Select all components to reset */
	reset = ICP1K_ASIC_CTRL_RX_RESET |
	        ICP1K_ASIC_CTRL_TX_RESET |
	        ICP1K_ASIC_CTRL_DMA_RESET |
	        ICP1K_ASIC_CTRL_FIFO_RESET |
	        ICP1K_ASIC_CTRL_NETWORK_RESET |
	        ICP1K_ASIC_CTRL_HOST_RESET;

	/* Actually do the selected resets */
	reset |= ICP1K_ASIC_CTRL_RESET;

	/* Reload EEPROM contents after reset */
	reset |= ICP1K_ASIC_CTRL_AUTOINIT;

	/* Issue reset */
	writel ( reset, icp->regs + ICP1K_ASIC_CTRL );

	/* Datasheet suggests waiting 5ms before further register accesses */
	mdelay ( 5 );

	while ( timeout-- ) {
		reset = readl ( icp->regs + ICP1K_ASIC_CTRL );

		/* Check if reset finished */
		if ( ! ( reset & ICP1K_ASIC_CTRL_RESET_BUSY ) )
			return 0;

		udelay ( 1 );
	}

	DBGC ( icp, "ICP %p reset timeout\n", icp );
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
static void icp1k_check_link ( struct net_device *netdev ) {
	struct icp1k_nic *icp = netdev->priv;
	uint8_t phy_ctrl;

	/* Read PHY Control register */
	phy_ctrl = readb ( icp->regs + ICP1K_PHY_CTRL );

	/* Check link state */
	if ( ( phy_ctrl & ICP1K_PHY_LINKSPEED_MSK ) == 0 ) {
		DBGC ( icp, "ICP1K %p Link down\n", icp );
		netdev_link_down ( netdev );
	} else {
		DBGC ( icp, "ICP1K %p Link up\n", icp );
		netdev_link_up ( netdev );
	}
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Initialise descriptor ring
 *
 * @v ring		Descriptor ring
 * @v count		Number of descriptors
 * @v reg		Register of ring base
 */
static void icp1k_init_ring ( struct icp1k_ring *ring, unsigned int count,
                              unsigned int reg ) {
	ring->reg = reg;
	ring->count = count;
	ring->size = ( count * sizeof ( struct icp1k_desc ) );
}

static int icp1k_create_ring ( struct icp1k_nic *icp, struct icp1k_ring *ring ) {
	unsigned int i;
	uint64_t addr;
	struct icp1k_desc *desc;

	/* Alocate ring */
	ring->desc = malloc_dma ( ring->size, ICP1K_DMA_ALIGN );

	if ( ! ring->desc )
		return -ENOMEM;

	memset ( ring->desc, 0, ring->size );

	/* Initialise all the descriptors */
	for (i = 0; i < ring->count; i++ ) {
		struct icp1k_desc *next;

		desc = &ring->desc[i];
		next = &ring->desc[i % ring->count];

		desc->next = cpu_to_le64 ( virt_to_bus ( next ) );
///		desc->status = 0ULL;
		desc->status = ICP1K_TX_DESC_DONE;
		desc->data = 0ULL;
	}

	addr = cpu_to_le64 ( virt_to_bus ( ring->desc ) );

	/* Write ring address to card */
	writel ( 0, ring->reg + icp->regs + ICP1K_REG_HI );
///	writel ( addr, ring->reg + icp->regs + ICP1K_REG_LO );

	return 0;
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int icp1k_open ( struct net_device *netdev ) {
	struct icp1k_nic *icp = netdev->priv;
	int rc;

	/* Enable interrupt notofications */
	writew ( ICP1K_INT_LINK_EVENT, icp->regs + ICP1K_INT_ENABLE );

	/* Create transmit descriptor ring */
	rc = icp1k_create_ring ( icp, &icp->tx );
	if ( ! rc )
		return rc;

	DBGC ( icp, "SKELETON %p does not yet support open\n", icp );
	return 0;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void icp1k_close ( struct net_device *netdev ) {
	struct icp1k_nic *icp = netdev->priv;

	DBGC ( icp, "SKELETON %p does not yet support close\n", icp );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int icp1k_transmit ( struct net_device *netdev,
			       struct io_buffer *iobuf ) {
	struct icp1k_nic *icp = netdev->priv;
	struct icp1k_ring *ring = &icp->tx;
	struct icp1k_desc *desc;
	uint64_t tmp;
	int tx_idx = ( ring->prod++ % ICP1K_NUM_TX_DESC );

	desc = &ring->desc[tx_idx];

	DBGC2 ( icp, "ICP1K %p transmitting iobuf %p tx_idx %d\n", icp, iobuf,
	        tx_idx );

	/* Fill transmit descriptor */
	tmp = ( (unsigned long long) virt_to_bus ( iobuf->data ) | ( (unsigned long long) iob_len ( iobuf ) << 48 ) );
	desc->data = cpu_to_le64 ( tmp );
	desc->status = ICP1K_TX_DESC_INTR | ICP1K_DESC_1FRAG;

	/* Trigger transmit */
	tmp = cpu_to_le64 ( virt_to_bus ( desc ) );
	writel ( tmp, icp->regs + ICP1K_TX_RING );

	return 0;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void icp1k_poll ( struct net_device *netdev ) {
	struct icp1k_nic *icp = netdev->priv;
	uint16_t int_status;

	/* Reading from Status/Ack register ACKS the interrupt and disables it
	 * in ICP1K_INT_ENABLE
	 */
	int_status = readw ( icp->regs + ICP1K_INT_STATUS_ACK );
///	DBGC ( icp, "ICP %p interrupt status %04x\n", icp, int_status );

	/* Handle link state change */
	if ( int_status & ICP1K_INT_LINK_EVENT )
		icp1k_check_link ( netdev );

	/* Not yet implemented */
	( void ) icp;
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void icp1k_irq ( struct net_device *netdev, int enable ) {
	struct icp1k_nic *icp = netdev->priv;

	DBGC ( icp, "SKELETON %p does not yet support interrupts\n", icp );
	( void ) enable;
}

/** IC+ 1000 network device operations */
static struct net_device_operations icp1k_operations = {
	.open		= icp1k_open,
	.close		= icp1k_close,
	.transmit	= icp1k_transmit,
	.poll		= icp1k_poll,
	.irq		= icp1k_irq,
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
static int icp1k_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct icp1k_nic *icp;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *icp ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &icp1k_operations );
	icp = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( icp, 0, sizeof ( *icp ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	icp->regs = ioremap ( pci->membase, ICP1K_BAR_SIZE );

	/* Reset the NIC */
	if ( ( rc = icp1k_reset ( icp ) ) != 0 )
		goto err_reset;

	/* Initialise and reset MII interface */
	mii_init ( &icp->mii, &icp1k_mii_operations );
	if ( ( rc = mii_reset ( &icp->mii ) ) != 0 ) {
		DBGC ( icp, "SKELETON %p could not reset MII: %s\n",
		       icp, strerror ( rc ) );
		goto err_mii_reset;
	}

	/* Initialise descriptor rings */
	icp1k_init_ring ( &icp->tx, ICP1K_NUM_TX_DESC, ICP1K_TX_RING );
	icp1k_init_ring ( &icp->rx, ICP1K_NUM_RX_DESC, 0 );

	netdev->hw_addr[0] = 0x70;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	icp1k_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_mii_reset:
	icp1k_reset ( icp );
 err_reset:
	iounmap ( icp->regs );
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
static void icp1k_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct icp1k_nic *icp = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset card */
	icp1k_reset ( icp );

	/* Free network device */
	iounmap ( icp->regs );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** IC+ 1000 PCI device IDs */
static struct pci_device_id icp1k_nics[] = {
	PCI_ROM ( 0x13f0, 0x1023, "icp",	"IC+1000", 0 ),
};

/** IC+ 1000 PCI driver */
struct pci_driver icp1k_driver __pci_driver = {
	.ids = icp1k_nics,
	.id_count = ( sizeof ( icp1k_nics ) / sizeof ( icp1k_nics[0] ) ),
	.probe = icp1k_probe,
	.remove = icp1k_remove,
};
