/*
 * Copyright (C) 2012 Michael Brown <mbrown@fensystems.co.uk>.
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

	DBGC ( icp, "SKELETON %p does not yet support link state\n", icp );
	netdev_link_err ( netdev, -ENOTSUP );
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
static int icp1k_open ( struct net_device *netdev ) {
	struct icp1k_nic *icp = netdev->priv;

	DBGC ( icp, "SKELETON %p does not yet support open\n", icp );
	return -ENOTSUP;
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

	DBGC ( icp, "SKELETON %p does not yet support transmit\n", icp );
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void icp1k_poll ( struct net_device *netdev ) {
	struct icp1k_nic *icp = netdev->priv;

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
	PCI_ROM ( 0x5ce1, 0x5ce1, "icp",	"IC+1000", 0 ),
};

/** IC+ 1000 PCI driver */
struct pci_driver icp1k_driver __pci_driver = {
	.ids = icp1k_nics,
	.id_count = ( sizeof ( icp1k_nics ) / sizeof ( icp1k_nics[0] ) ),
	.probe = icp1k_probe,
	.remove = icp1k_remove,
};
