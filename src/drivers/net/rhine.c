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

	DBGC ( rhn, "RHINE %p MII read reg %d\n", rhn, reg );
	
	writeb ( reg, rhn->regs + RHINE_MII_PA );
	rhine_setbit( rhn->regs + RHINE_MII_CR, RHINE_MII_CR_RDEN );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( (readb ( rhn->regs + RHINE_MII_CR ) & RHINE_MII_CR_RDEN ) == 0 )
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
	struct rhine_nic *rhn = container_of(mii, struct rhine_nic, mii);
	int timeout = RHINE_TIMEOUT_US;

	DBGC ( rhn, "RHINE %p MII write reg %d data 0x%04x\n", rhn, reg, data );

	writeb ( reg, rhn->regs + RHINE_MII_PA );
	writew ( data, rhn->regs + RHINE_MII_RDWR );
	rhine_setbit ( rhn->regs + RHINE_MII_CR, RHINE_MII_CR_WREN );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( rhn->regs + RHINE_MII_CR ) & RHINE_MII_CR_WREN ) == 0 )
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
 * @v rhn		Rhine device
 * @ret rc		Return status code
 */
static int rhine_reset ( struct rhine_nic *rhn ) {
	int timeout = RHINE_TIMEOUT_US;

	DBGC ( rhn, "RHINE %p reset\n", rhn );

	rhine_setbit ( rhn->regs + RHINE_CR1, RHINE_CR1_RESET );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( rhn->regs + RHINE_CR1 ) & RHINE_CR1_RESET ) == 0 )
			return 0;
	}
	
	DBGC ( rhn, "RHINE %p reset timeout\n", rhn );
	return -ETIMEDOUT;
}

/**
 * Reload EEPROM contents
 *
 * @v rhn		Rhine device
 */
static int rhine_reload_eeprom ( struct rhine_nic *rhn ) {
	int timeout = RHINE_TIMEOUT_US;

	rhine_setbit( rhn->regs + RHINE_EEPROM_CTRL, RHINE_EEPROM_CTRL_RELOAD );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( rhn->regs + RHINE_EEPROM_CTRL )
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

	DBGC ( rhn, "RHINE %p check link\n", rhn );

	if ( readb ( rhn->regs + RHINE_MII_SR ) & RHINE_MII_SR_LINKNWAY ) {
		DBGC ( rhn, "RHINE %p link down\n", rhn );
		netdev_link_down ( netdev );
	} else {
		DBGC ( rhn, "RHINE %p link up\n", rhn );
		netdev_link_up ( netdev );
	}

	if ( readb ( rhn->regs + RHINE_MII_SR ) & RHINE_MII_SR_PHYERR )
		netdev_link_err ( netdev, -EINVAL );
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
static int rhine_open ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;

	DBGC ( rhn, "RHINE %p does not yet support open\n", rhn );
	return -ENOTSUP;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void rhine_close ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;

	DBGC ( rhn, "RHINE %p does not yet support close\n", rhn );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int rhine_transmit ( struct net_device *netdev, struct io_buffer *iobuf ) {
	struct rhine_nic *rhn = netdev->priv;

	DBGC ( rhn, "RHINE %p does not yet support transmit\n", rhn);
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void rhine_poll ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;

	rhine_check_link ( netdev );

	/* Not yet implemented */
	( void ) rhn;
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void rhine_irq ( struct net_device *netdev, int enable ) {
	struct rhine_nic *nic = netdev->priv;

	DBGC ( nic, "RHINE %p does not yet support interrupts\n", nic );
	( void ) enable;
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

	/* Map registers */
	rhn->regs = ioremap ( pci->membase, RHINE_BAR_SIZE );

	/* Reset the NIC */
	if ( ( rc = rhine_reset ( rhn ) ) != 0 )
		goto err_reset;

	/* Reload EEPROM */
	if ( ( rc = rhine_reload_eeprom ( rhn ) ) != 0 )
		goto err_reset;

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

	DBGC ( rhn, "RHINE PHY vendor id: %04x\n", rhine_mii_read ( &rhn->mii, 0x02 ) );
	DBGC ( rhn, "RHINE PHY device id: %04x\n", rhine_mii_read ( &rhn->mii, 0x03 ) );

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	rhine_check_link ( netdev );

	return 0;

	unregister_netdev( netdev );
 err_register_netdev:
 err_mii_reset:
	rhine_reset ( rhn );
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
	rhine_reset ( nic );

	/* Free network device */
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** Rhine PCI device IDs */
static struct pci_device_id rhine_nics[] = {
	PCI_ROM ( 0x1106, 0x3106, "vt6105m",	"VIA VT6105M", 0 ),
};

/** Rhine PCI driver */
struct pci_driver rhine_driver __pci_driver = {
	.ids = rhine_nics,
	.id_count = ( sizeof ( rhine_nics ) / sizeof ( rhine_nics[0] ) ),
	.probe = rhine_probe,
	.remove = rhine_remove,
};
