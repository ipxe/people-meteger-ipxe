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

/** @file
 *
 * bnx2 network driver
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
static int bnx2_mii_read ( struct mii_interface *mii, unsigned int reg ) {
	struct bnx2_nic *bnx2 =
		container_of ( mii, struct bnx2_nic, mii );

	DBGC ( bnx2, "bnx2 %p does not yet support MII read\n", bnx2 );
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
static int bnx2_mii_write ( struct mii_interface *mii, unsigned int reg,
				unsigned int data) {
	struct bnx2_nic *bnx2 =
		container_of ( mii, struct bnx2_nic, mii );

	DBGC ( bnx2, "bnx2 %p does not yet support MII write\n", bnx2 );
	( void ) reg;
	( void ) data;
	return -ENOTSUP;
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

	DBGC ( bnx2, "bnx2 %p does not yet support reset\n", bnx2 );
	return -ENOTSUP;
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

	DBGC ( bnx2, "bnx2 %p does not yet support link state\n", bnx2 );
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
static int bnx2_open ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;

	DBGC ( bnx2, "bnx2 %p does not yet support open\n", bnx2 );
	return -ENOTSUP;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void bnx2_close ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;

	DBGC ( bnx2, "bnx2 %p does not yet support close\n", bnx2 );
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

	DBGC ( bnx2, "bnx2 %p does not yet support transmit\n", bnx2 );
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void bnx2_poll ( struct net_device *netdev ) {
	struct bnx2_nic *bnx2 = netdev->priv;

	/* Not yet implemented */
	( void ) bnx2;
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void bnx2_irq ( struct net_device *netdev, int enable ) {
	struct bnx2_nic *bnx2 = netdev->priv;

	DBGC ( bnx2, "bnx2 %p does not yet support interrupts\n", bnx2 );
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
	uint32_t misc_id;
	
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

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	bnx2->regs = ioremap ( pci->membase, BNX2_BAR_SIZE );

	misc_id = readl (bnx2->regs + BNX2_MISC_ID);
	DBGC ( bnx2, "BCM%04X (rev %c%d) detected\n",
				 ( unsigned int ) ( misc_id & BNX2_MISC_ID_CHIP_NUM ) >> 16,
				 ( int ) ( ( misc_id & BNX2_MISC_ID_CHIP_REV ) >> 12 ) + 'A',
				 ( int ) ( misc_id & BNX2_MISC_ID_CHIP_METAL ) >> 4 );

	/* Reset the NIC */
	if ( ( rc = bnx2_reset ( bnx2 ) ) != 0 )
		goto err_reset;

	/* Initialise and reset MII interface */
	mii_init ( &bnx2->mii, &bnx2_mii_operations );
	if ( ( rc = mii_reset ( &bnx2->mii ) ) != 0 ) {
		DBGC ( bnx2, "bnx2 %p could not reset MII: %s\n",
		       bnx2, strerror ( rc ) );
		goto err_mii_reset;
	}

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	bnx2_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_mii_reset:
	bnx2_reset ( bnx2 );
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
