/*
 * Copyright (C) 2012 Adrian Jamróz <adrian.jamroz@gmail.com>
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
#include "velocity.h"

#define	velocity_setbit(_reg, _mask)	writeb ( readb ( _reg ) | _mask, _reg )

/** @file
 *
 * VIA Velocity network driver
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
static int velocity_mii_read ( struct mii_interface *mii, unsigned int reg ) {
	struct velocity_nic *vlc =
		container_of ( mii, struct velocity_nic, mii );

	DBGC ( vlc, "VELOCITY %p does not yet support MII read\n", vlc );
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
static int velocity_mii_write ( struct mii_interface *mii, unsigned int reg,
				unsigned int data) {
	struct velocity_nic *vlc =
		container_of ( mii, struct velocity_nic, mii );

	DBGC ( vlc, "VELOCITY %p does not yet support MII write\n", vlc );
	( void ) reg;
	( void ) data;
	return -ENOTSUP;
}

/** Velocity MII operations */
static struct mii_operations velocity_mii_operations = {
	.read = velocity_mii_read,
	.write = velocity_mii_write,
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
 * @v vlc		Velocity device
 * @ret rc		Return status code
 */
static int velocity_reset ( struct velocity_nic *vlc ) {
	int timeout = VELOCITY_TIMEOUT_US;

	DBGC ( vlc, "VELOCITY %p reset\n", vlc );

	velocity_setbit ( vlc->regs + VELOCITY_CRS1, VELOCITY_CRS1_SFRST );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( vlc->regs + VELOCITY_CRS1 ) & VELOCITY_CRS1_SFRST ) == 0 )
			return 0;
	}

	return -EINVAL;
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
static void velocity_check_link ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;

	DBGC ( vlc, "VELOCITY %p does not yet support link state\n", vlc );
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
static int velocity_open ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;

	DBGC ( vlc, "VELOCITY %p does not yet support open\n", vlc );
	return -ENOTSUP;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void velocity_close ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;

	DBGC ( vlc, "VELOCITY %p does not yet support close\n", vlc );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int velocity_transmit ( struct net_device *netdev,
			       struct io_buffer *iobuf ) {
	struct velocity_nic *vlc = netdev->priv;

	DBGC ( vlc, "VELOCITY %p does not yet support transmit\n", vlc );
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void velocity_poll ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;

	/* Not yet implemented */
	( void ) vlc;
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void velocity_irq ( struct net_device *netdev, int enable ) {
	struct velocity_nic *vlc = netdev->priv;

	DBGC ( vlc, "VELOCITY %p does not yet support interrupts\n", vlc );
	( void ) enable;
}

/** Velocity network device operations */
static struct net_device_operations velocity_operations = {
	.open		= velocity_open,
	.close		= velocity_close,
	.transmit	= velocity_transmit,
	.poll		= velocity_poll,
	.irq		= velocity_irq,
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
static int velocity_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct velocity_nic *vlc;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *vlc ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &velocity_operations );
	vlc = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( vlc, 0, sizeof ( *vlc ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	vlc->regs = ioremap ( pci->membase, VELOCITY_BAR_SIZE );

	/* Reset the NIC */
	if ( ( rc = velocity_reset ( vlc ) ) != 0 )
		goto err_reset;

	/* Initialise and reset MII interface */
	mii_init ( &vlc->mii, &velocity_mii_operations );
	if ( ( rc = mii_reset ( &vlc->mii ) ) != 0 ) {
		DBGC ( vlc, "VELOCITY %p could not reset MII: %s\n",
		       vlc, strerror ( rc ) );
		goto err_mii_reset;
	}

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	velocity_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_mii_reset:
	velocity_reset ( vlc );
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
static void velocity_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct velocity_nic *vlc = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset card */
	velocity_reset ( vlc );

	/* Free network device */
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** Velocity PCI device IDs */
static struct pci_device_id velocity_nics[] = {
	PCI_ROM ( 0x5ce1, 0x5ce1, "vlc",	"Velocity", 0 ),
};

/** Velocity PCI driver */
struct pci_driver velocity_driver __pci_driver = {
	.ids = velocity_nics,
	.id_count = ( sizeof ( velocity_nics ) / sizeof ( velocity_nics[0] ) ),
	.probe = velocity_probe,
	.remove = velocity_remove,
};
