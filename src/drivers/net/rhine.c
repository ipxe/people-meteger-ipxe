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

	if ( readb ( rhn->regs + RHINE_MII_SR ) & RHINE_MII_SR_LINKNWAY )
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
	if ( ! rhn->rx_ring )
		return -ENOMEM;

	memset ( rhn->rx_ring, 0, RHINE_RXDESC_SIZE );

	for ( i = 0; i < RHINE_RXDESC_NUM; i++ )
		rhn->rx_ring[i].next = virt_to_bus( 
		    &rhn->rx_ring[ (i + 1) % RHINE_RXDESC_NUM] );

	DBGC ( rhn, "RHINE %p RX ring start address: %p\n", rhn, rhn->rx_ring );

	/* Program RX ring start address */
	writel ( virt_to_bus ( rhn->rx_ring ), rhn->regs + RHINE_RXQUEUE_BASE );

	/* Allocate TX descriptor ring */
	rhn->tx_prod = 0;
	rhn->tx_cons = 0;
	rhn->tx_ring = malloc_dma ( RHINE_TXDESC_SIZE, RHINE_RING_ALIGN );
	if ( ! rhn->tx_ring )
		return -ENOMEM;

	memset ( rhn->tx_ring, 0, RHINE_TXDESC_SIZE );

	for ( i = 0; i < RHINE_TXDESC_NUM; i++ )
		rhn->tx_ring[i].next = virt_to_bus (
		    &rhn->tx_ring[ (i + 1) % RHINE_TXDESC_NUM] );

	DBGC ( rhn, "RHINE %p TX ring start address: %p\n", rhn, rhn->tx_ring );

	/* Program TX ring start address */
	writel ( virt_to_bus ( rhn->tx_ring ), rhn->regs + RHINE_TXQUEUE_BASE );

	return 0;
}

static int rhine_refill_rx ( struct rhine_nic *rhn )
{
	struct rhine_descriptor *desc;
	struct io_buffer *iobuf;
	int rx_idx, i = 0;

	while ( ( rhn->rx_prod - rhn->rx_cons ) < RHINE_RXDESC_NUM ) {
		iobuf = alloc_iob ( RHINE_RX_MAX_LEN );
		if ( ! iobuf )
			return 0;

		rx_idx = ( rhn->rx_prod++ % RHINE_RXDESC_NUM );
		desc = &rhn->rx_ring[rx_idx];
		desc->buffer = virt_to_bus ( iobuf-> data );
		desc->des0 = RHINE_DES0_OWN;
		desc->des1 = RHINE_DES1_SIZE ( RHINE_RX_MAX_LEN - 1 )
		    | RHINE_DES1_CHAIN | RHINE_DES1_IC;

		rhn->rx_buffs[rx_idx] = iobuf;
		i++;

		//DBGC ( rhn, "RHINE %p refilled idx=%d\n", rhn,
		//    rx_idx );
	}

	if (i > 0)
		DBGC ( rhn, "RHINE %p refilled %d RX descriptors\n", rhn, i );
	return 0;
}

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int rhine_open ( struct net_device *netdev ) {
	struct rhine_nic *rhn = netdev->priv;
	int rc;

	DBGC ( rhn, "RHINE %p open\n", rhn );
	DBGC ( rhn, "RHINE %p regs at: %p\n", rhn, rhn->regs );

	if ( ( rc = rhine_reset ( rhn ) ) != 0 )
		return rc;

	if ( ( rc = rhine_alloc_rings ( rhn ) ) != 0 )
		return rc;

	if ( ( rc = rhine_refill_rx ( rhn ) ) != 0 )
		goto err_refill_rx;

	writeb ( RHINE_RCR_PHYS_ACCEPT | RHINE_RCR_BCAST_ACCEPT |
	    RHINE_RCR_RUNT_ACCEPT, rhn->regs + RHINE_RCR );

	writeb ( RHINE_CR0_STARTNIC | RHINE_CR0_RXEN | RHINE_CR0_TXEN,
	    rhn->regs + RHINE_CR0 );

	writeb ( RHINE_CR1_RXPOLL | RHINE_CR1_FDX,
	    rhn->regs + RHINE_CR1 );

	return 0;

err_refill_rx:
	free_dma ( rhn->rx_ring, RHINE_RXDESC_SIZE );
	free_dma ( rhn->tx_ring, RHINE_TXDESC_SIZE );
	return rc;
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
	struct rhine_descriptor *desc;
	int tx_idx;

	DBGC ( rhn, "RHINE %p transmit\n", rhn);

	iob_pad ( iobuf, ETH_ZLEN );

	tx_idx = ( rhn->tx_prod++ % RHINE_TXDESC_NUM );
	desc = &rhn->tx_ring[tx_idx];
	desc->buffer = cpu_to_le32 ( virt_to_bus ( iobuf->data ) );
	desc->des0 = cpu_to_le32 ( RHINE_DES0_OWN );
	desc->des1 = cpu_to_le32 ( RHINE_DES1_IC | RHINE_TDES1_STP
	    | RHINE_TDES1_EDP | RHINE_DES1_CHAIN 
	    | RHINE_DES1_SIZE ( iob_len ( iobuf ) ) );
	
	DBGC ( rhn, "RHINE %p tx_prod=%d desc=%p iobuf=%p len=%d ethertype=%02x\n", rhn,
	    tx_idx, desc, iobuf->data, iob_len ( iobuf ), *((unsigned short *)iobuf->data + 6) );

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

		if ( le32_to_cpu ( desc->des0 ) & RHINE_DES0_OWN)
			return;

		DBGC ( rhn, "RHINE %p got packet on idx=%d (prod=%d) ethertype=%02x\n",
		    rhn, rx_idx, rhn->rx_prod % RHINE_RXDESC_NUM, *((unsigned short *)rhn->rx_buffs[rx_idx]->data + 6) );

		iobuf = rhn->rx_buffs[rx_idx];
		iob_put ( iobuf, ( RHINE_DES0_GETSIZE (
		    le32_to_cpu ( desc->des0 ) ) ) );

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
	
		DBGC ( rhn, "RHINE %p poll_tx cons=%d prod=%d tsr=%04x\n",
		    rhn, tx_idx, rhn->tx_prod % RHINE_TXDESC_NUM, desc->des0 & 0xffff );
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

	rhine_check_link ( netdev );

	isr0 = readb ( rhn->regs + RHINE_ISR0 );
	isr1 = readb ( rhn->regs + RHINE_ISR1 );

#if 0
	DBGC ( rhn, "RHINE %p ISR0=%02x ISR1=%02x\n", rhn, isr0, isr1 );
#endif

	writeb ( 0, rhn->regs + RHINE_ISR0 );
	writeb ( 0, rhn->regs + RHINE_ISR1 );

	if (isr0 & RHINE_ISR0_TXDONE)
		rhine_poll_tx ( rhn );

	if (isr0 & RHINE_ISR0_RXDONE)
		rhine_poll_rx ( rhn );

	if (isr0 & RHINE_ISR0_RXERR)
		DBGC ( rhn, "RHINE %p RXERR\n", rhn );

	if (isr0 & RHINE_ISR0_RXRINGERR)
		DBGC ( rhn, "RHINE %p RXRINGERR\n", rhn );

	rhine_refill_rx ( rhn );
	( void )isr1;
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
	rhn->netdev = netdev;
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
