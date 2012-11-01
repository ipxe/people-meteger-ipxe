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

static int velocity_autopoll_stop ( struct velocity_nic *vlc )
{
	int timeout = VELOCITY_TIMEOUT_US;

	/* Disable MII auto polling */
	writeb ( 0, vlc->regs + VELOCITY_MIICR );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( readb ( vlc->regs + VELOCITY_MIISR ) & 
		    VELOCITY_MIISR_IDLE )
			return 0;	
	}


	DBGC ( vlc, "MII autopoll stop timeout\n" );
	return -ETIMEDOUT;
}

static int velocity_autopoll_start ( struct velocity_nic *vlc )
{
	int timeout = VELOCITY_TIMEOUT_US;

	/* Disable MII auto polling */
	writeb ( VELOCITY_MIICR_MAUTO, vlc->regs + VELOCITY_MIICR );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( vlc->regs + VELOCITY_MIISR ) & 
		    VELOCITY_MIISR_IDLE ) == 0 )
			return 0;	
	}


	DBGC ( vlc, "MII autopoll start timeout\n" );
	return -ETIMEDOUT;
}

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
	int timeout = VELOCITY_TIMEOUT_US;
	int result;

	DBGC2 ( vlc, "VELOCITY %p MII read reg %d\n", vlc, reg );
	
	velocity_autopoll_stop ( vlc );

	writeb ( reg, vlc->regs + VELOCITY_MIIADDR );
	velocity_setbit( vlc->regs + VELOCITY_MIICR, VELOCITY_MIICR_RCMD );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( vlc->regs + VELOCITY_MIICR ) &
		    VELOCITY_MIICR_RCMD ) == 0 ) {
			result = readw ( vlc->regs + VELOCITY_MIIDATA );
			velocity_autopoll_start ( vlc );
			return result;
		}	
	}

	velocity_autopoll_start ( vlc );

	DBGC ( vlc, "MII read timeout\n" );
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
static int velocity_mii_write ( struct mii_interface *mii, unsigned int reg,
				unsigned int data) {
	struct velocity_nic *vlc =
		container_of ( mii, struct velocity_nic, mii );
	int timeout = VELOCITY_TIMEOUT_US;

	DBGC2 ( vlc, "VELOCITY %p MII write reg %d data 0x%04x\n", vlc, reg, data );

	DBGC2 ( vlc, "VELOCITY %p regs at 0x%p\n", vlc, vlc->regs );

	velocity_autopoll_stop ( vlc );

	writeb ( reg, vlc->regs + VELOCITY_MIIADDR );
	writew ( data, vlc->regs + VELOCITY_MIIDATA );
	velocity_setbit ( vlc->regs + VELOCITY_MIICR, VELOCITY_MIICR_WCMD );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( vlc->regs + VELOCITY_MIICR ) 
		    & VELOCITY_MIICR_WCMD ) == 0 ) {
			velocity_autopoll_start ( vlc );
			return 0;
		}
	}

	velocity_autopoll_start ( vlc );

	DBGC ( vlc, "MII write timeout\n" );
	return -ETIMEDOUT;
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
 * reload eeprom contents
 *
 * @v vlc		Velocity device
 */
static int velocity_reload_eeprom ( struct velocity_nic *vlc ) {
	int timeout = VELOCITY_TIMEOUT_US;

	velocity_setbit( vlc->regs + VELOCITY_EECSR, VELOCITY_EECSR_RELOAD );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( vlc->regs + VELOCITY_EECSR )
		    & VELOCITY_EECSR_RELOAD ) == 0 )
			return 0;
	}

	DBGC ( vlc, "VELOCITY %p EEPROM reload timeout\n", vlc );
	return -ETIMEDOUT;
}

/*
 * Reset hardware
 *
 * @v vlc		Velocity device
 * @ret rc		Return status code
 */
static int velocity_reset ( struct velocity_nic *vlc ) {
	int timeout = VELOCITY_TIMEOUT_US;

	DBGC ( vlc, "VELOCITY %p reset\n", vlc );

	/* NOTE: datasheet reset procedure(p. 41) disables some sticky bits,
	 * changes power states, etc.
	 */

	velocity_setbit ( vlc->regs + VELOCITY_CRS1, VELOCITY_CRS1_SFRST );

	while ( timeout-- ) {
		udelay ( 1 );
		if ( ( readb ( vlc->regs + VELOCITY_CRS1 ) & VELOCITY_CRS1_SFRST ) == 0 )
			return 0;
	}

	/* NOTE: if normal reset times out, datasheet suggests forced reset and
	 * returning success unconditionally after 2ms */

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

	if ( readb ( vlc->regs + VELOCITY_PHYSTS0 ) & VELOCITY_PHYSTS0_LINK ) {
		netdev_link_up ( netdev );
		DBGC ( vlc, "VELOCITY %p link up\n", vlc );
	} else {
		netdev_link_down ( netdev );
		DBGC ( vlc, "VELOCITY %p link down\n", vlc );
	}

	velocity_autopoll_start ( vlc );
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

static int velocity_alloc_rings ( struct velocity_nic *vlc )
{
	/* Allocate RX descriptor ring */
	vlc->rx_prod = 0;
	vlc->rx_cons = 0;
	vlc->rx_commit = 0;
	vlc->rx_ring = malloc_dma ( VELOCITY_RXDESC_SIZE, VELOCITY_RING_ALIGN );
	if ( ! vlc->rx_ring )
		return -ENOMEM;

	memset ( vlc->rx_ring, 0, VELOCITY_RXDESC_SIZE );

	DBGC ( vlc, "VELOCITY %p RX ring start address: %p\n", vlc, vlc->rx_ring );
	DBGC ( vlc, "VELOCITY %p RX ring phys: 0x%08lx\n", vlc, virt_to_bus ( vlc->rx_ring ) );
	/* Allocate TX descriptor ring */
	vlc->tx_prod = 0;
	vlc->tx_cons = 0;
	vlc->tx_ring = malloc_dma ( VELOCITY_TXDESC_SIZE, VELOCITY_RING_ALIGN );
	if ( ! vlc->tx_ring )
		return -ENOMEM;

	memset ( vlc->tx_ring, 0, VELOCITY_TXDESC_SIZE );

	writel ( virt_to_bus ( vlc->rx_ring ), vlc->regs + VELOCITY_RXDESC_ADDR_LO );
	writew ( VELOCITY_RXDESC_NUM - 1, vlc->regs + VELOCITY_RXDESCNUM );
	writel ( virt_to_bus ( vlc->tx_ring ), vlc->regs + VELOCITY_TXDESC_ADDR_LO0 );
	writew ( VELOCITY_TXDESC_NUM - 1, vlc->regs + VELOCITY_TXDESCNUM );

	DBGC ( vlc, "VELOCITY %p TX ring start address: %p\n", vlc, vlc->tx_ring );
	DBGC ( vlc, "VELOCITY %p TX ring phys: 0x%08lx\n", vlc, virt_to_bus ( vlc->tx_ring ) );

	return 0;
}

static void velocity_refill_rx ( struct velocity_nic *vlc )
{
	struct velocity_rx_descriptor *desc;
	struct io_buffer *iobuf;
	int rx_idx, i = 0;

	while ( ( vlc->rx_prod - vlc->rx_cons ) < VELOCITY_RXDESC_NUM ) {
		iobuf = alloc_iob ( VELOCITY_RX_MAX_LEN );
		if ( ! iobuf )
			break;

		rx_idx = ( vlc->rx_prod++ % VELOCITY_RXDESC_NUM );
		desc = &vlc->rx_ring[rx_idx];
#if 0
		desc->des0 = cpu_to_le32 ( VELOCITY_DES0_OWN );
#endif
		desc->des1 = 0;
		desc->addr = cpu_to_le32 ( virt_to_bus ( iobuf->data ) );
		desc->des2 = cpu_to_le32 ( 
		    VELOCITY_DES2_SIZE( VELOCITY_RX_MAX_LEN - 1 ) |
		    VELOCITY_DES2_IC );

		vlc->rx_buffs[rx_idx] = iobuf;
		i++;

		/* We can only send multiples of 4 to the NIC. HW requirement */
		if ( rx_idx % 4 == 3 ) {
			int j;
			for (j = 0; j < 4; j++) {
				desc = &vlc->rx_ring[rx_idx - j];
				desc->des0 = cpu_to_le32 ( VELOCITY_DES0_OWN );
				/* NOTE: should this really be 4? */
				vlc->rx_commit += 4;
			}
		}
	}

	if (vlc->rx_commit) {
		writew ( vlc->rx_commit, vlc->regs + VELOCITY_RXDESC_RESIDUECNT );
		vlc->rx_commit = 0;
	}

	if (i > 0)
		DBGC2 ( vlc, "VELOCITY %p refilled %d RX descriptors\n", vlc, i );
}


/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int velocity_open ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;
	int rc;

	DBGC ( vlc, "VELOCITY %p open\n", vlc );
	DBGC ( vlc, "VELOCITY %p regs at: %p\n", vlc, vlc->regs );

	/* NOTE: alreade done in velocity_probe(). Why again? */
	if ( ( rc = velocity_reset ( vlc ) ) != 0 )
		return rc;

	if ( ( rc = velocity_alloc_rings ( vlc ) ) != 0 )
		return rc;

	velocity_refill_rx ( vlc );

	writew ( VELOCITY_TXQCSRS_RUN0, vlc->regs + VELOCITY_TXQCSRS );
	writew ( VELOCITY_RXQCSRS_RUN | VELOCITY_RXQCSRS_WAK,
	    vlc->regs + VELOCITY_RXQCSRS );

	/* NOTE: will this actually trigger interrupts or only flag them in a register? */
	/* Enable interrupts */
	writeb ( 0xff, vlc->regs + VELOCITY_IMR0 );
	writeb ( 0xff, vlc->regs + VELOCITY_IMR1 );

	/* NOTE: we should set multicast filter to -1UL, enable promisc mode here */

	/* NOTE: test if this is sufficient */
	writeb ( RHINE_RCR_PROMISC, vlc->regs + VELOCITY_RCR );
#if 0
	/* NOTE: if not, try: */
	writeb ( RHINE_RCR_BCAST_ACCEPT |
	         RHINE_RCR_BCAST_ACCEPT |
	         RHINE_RCR_MCAST_ACCEPT,
	         vls->regs + VELOCITY_RCR );
#endif

	/* Start MAC */
	writeb ( VELOCITY_CRS0_STOP, vlc->regs + VELOCITY_CRC0 );
	writeb ( VELOCITY_CRS1_DPOLL, vlc->regs + VELOCITY_CRC0 );
	writeb ( VELOCITY_CRS0_START | VELOCITY_CRS0_TXON | VELOCITY_CRS0_RXON,
	    vlc->regs + VELOCITY_CRS0 );

	/* Set initial link state */
	velocity_check_link ( netdev );

	velocity_autopoll_start ( vlc );

	return 0;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void velocity_close ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;
	int i;

	DBGC ( vlc, "VELOCITY %p close\n", vlc );

	/* NOTE: do we have to disable tx/rx before we free the descriptors?
	 * if not, set descriptor number before setting address to 0? */

	/* Clear RX ring information */
	writel ( 0, vlc->regs + VELOCITY_RXDESC_ADDR_LO );
	writew ( 0, vlc->regs + VELOCITY_RXDESCNUM );

	/* Destroy RX ring */
	free_dma ( vlc->rx_ring, VELOCITY_RXDESC_SIZE );
	vlc->rx_ring = NULL;
	vlc->rx_prod = 0;
	vlc->rx_cons = 0;

	/* Discard receive buffers */
	for ( i = 0 ; i < VELOCITY_RXDESC_NUM ; i++ ) {
		if ( vlc->rx_buffs[i] )
			free_iob ( vlc->rx_buffs[i] );
		vlc->rx_buffs[i] = NULL;
	}

	/* Clear TX ring information */
	writel ( 0, vlc->regs + VELOCITY_TXDESC_ADDR_LO0 );
	writew ( 0, vlc->regs + VELOCITY_TXDESCNUM );

	/* Destroy TX ring */
	free_dma ( vlc->tx_ring, VELOCITY_TXDESC_SIZE );
	vlc->tx_ring = NULL;
	vlc->tx_prod = 0;
	vlc->tx_cons = 0;
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
	struct velocity_descriptor *desc;
	unsigned int tx_idx;

	DBGC ( vlc, "VELOCITY %p transmit\n", vlc);

	iob_pad ( iobuf, ETH_ZLEN );

	tx_idx = ( vlc->tx_prod++ % VELOCITY_TXDESC_NUM );
	desc = &vlc->tx_ring[tx_idx];
	desc->des0 = cpu_to_le32 ( VELOCITY_DES0_OWN |
	                           VELOCITY_DES2_SIZE ( iob_len ( iobuf ) ) );

	desc->des1 = cpu_to_le32 ( VELOCITY_DES1_FRAGS ( 1 ) |
	                           VELOCITY_DES1_NORM_PKT |
	                           VELOCITY_DES1_TIC );

	desc->frags[0].addr = cpu_to_le32 ( virt_to_bus ( iobuf->data ) );
	desc->frags[0].des2 = VELOCITY_DES2_SIZE ( iob_len ( iobuf ) );
	
	velocity_setbit ( vlc->regs + VELOCITY_TXQCSRS, VELOCITY_TXQCSRS_WAK0 );

	DBGC2 ( vlc, "VELOCITY %p tx_prod=%d desc=%p iobuf=%p len=%d\n", vlc,
	        tx_idx, desc, iobuf->data, iob_len ( iobuf ) );

	return 0;
}

static void velocity_poll_rx ( struct velocity_nic *vlc ) {
	struct velocity_rx_descriptor *desc;
	struct io_buffer *iobuf;
	int rx_idx;

	while ( vlc->rx_cons != vlc->rx_prod ) {
		rx_idx = ( vlc->rx_cons % VELOCITY_RXDESC_NUM );
		desc = &vlc->rx_ring[rx_idx];

		if ( le32_to_cpu ( desc->des0 ) & VELOCITY_DES0_OWN )
			return;

		DBGC2 ( vlc, "VELOCITY %p got packet on idx=%d (prod=%d)\n",
		    vlc, rx_idx, vlc->rx_prod % VELOCITY_RXDESC_NUM );

		iobuf = vlc->rx_buffs[rx_idx];

		iob_put ( iobuf, VELOCITY_DES0_RMBC ( 
		    le32_to_cpu ( desc->des0 ) ) );
		
		netdev_rx ( vlc->netdev, iobuf );
		
		vlc->rx_cons++;
	}
}

static void velocity_poll_tx ( struct velocity_nic *vlc ) {
	struct velocity_descriptor *desc;
	int tx_idx;

	while ( vlc->tx_cons != vlc->tx_prod ) {
		tx_idx = ( vlc->tx_cons % VELOCITY_TXDESC_NUM );
		desc = &vlc->tx_ring[tx_idx];

		if ( le32_to_cpu ( desc->des0 ) & VELOCITY_DES0_OWN )
			return;

		netdev_tx_complete_next ( vlc->netdev );

		DBGC2 ( vlc, "VELOCITY %p poll_tx cons=%d prod=%d tsr=%04x\n",
		    vlc, tx_idx, vlc->tx_prod % VELOCITY_TXDESC_NUM, ( desc->des0 & 0xffff ) );
		vlc->tx_cons++;
	}
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void velocity_poll ( struct net_device *netdev ) {
	struct velocity_nic *vlc = netdev->priv;
	uint8_t isr0, isr1;

	isr0 = readb ( vlc->regs + VELOCITY_ISR0 );
	isr1 = readb ( vlc->regs + VELOCITY_ISR1 );

	/* acknowledge interrupts */
	writeb ( 0xff, vlc->regs + VELOCITY_ISR0 );
	writeb ( 0xff, vlc->regs + VELOCITY_ISR1 );

	if ( isr0 & VELOCITY_ISR0_PTX0 ) {
		DBGC2 ( vlc, "VELOCITY %p TX interrupt\n", vlc );
		velocity_poll_tx ( vlc );
	}

	if ( isr0 & VELOCITY_ISR0_PRXI ) {
		DBGC2 ( vlc, "VELOCITY %p RX interrupt\n", vlc );
		velocity_poll_rx ( vlc );
	}

	if ( isr1 & VELOCITY_ISR1_SRCI ) {
		DBGC2 ( vlc, "VELOCITY %p link status interrupt\n", vlc );
		velocity_check_link ( netdev ); 
	}

	velocity_refill_rx ( vlc );
	writeb ( VELOCITY_RXQCSRS_RUN | VELOCITY_RXQCSRS_WAK,
	    vlc->regs + VELOCITY_RXQCSRS );
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void velocity_irq ( struct net_device *netdev, int enable ) {
	struct velocity_nic *vlc = netdev->priv;

	DBGC ( vlc, "VELOCITY %p interrupts %s\n", vlc,
	    enable ? "enable" : "disable" );

	/* NOTE: GintMsk1 in CR3 must be enabled for interrupts to work */

	if (enable) {
		/* Enable interrupts */
		writeb ( 0xff, vlc->regs + VELOCITY_IMR0 );
		writeb ( 0xff, vlc->regs + VELOCITY_IMR1 );
	} else {
		/* Disable interrupts */
		writeb ( 0, vlc->regs + VELOCITY_IMR0 );
		writeb ( 0, vlc->regs + VELOCITY_IMR1 );
	}
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

	/* NOTE: alloc_etherdev already does that */
	memset ( vlc, 0, sizeof ( *vlc ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	vlc->regs = ioremap ( pci->membase, VELOCITY_BAR_SIZE );
	vlc->netdev = netdev;

	/* Reset the NIC */
	if ( ( rc = velocity_reset ( vlc ) ) != 0 )
		goto err_reset;

	/* Reload EEPROM */
	if ( ( rc = velocity_reload_eeprom ( vlc ) ) != 0 )
		goto err_reset;

	netdev->hw_addr[0] = readb ( vlc->regs + VELOCITY_MAC0 );
	netdev->hw_addr[1] = readb ( vlc->regs + VELOCITY_MAC1 );
	netdev->hw_addr[2] = readb ( vlc->regs + VELOCITY_MAC2 );
	netdev->hw_addr[3] = readb ( vlc->regs + VELOCITY_MAC3 );
	netdev->hw_addr[4] = readb ( vlc->regs + VELOCITY_MAC4 );
	netdev->hw_addr[5] = readb ( vlc->regs + VELOCITY_MAC5 );

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

	return 0;

 err_register_netdev:
 err_mii_reset:
	velocity_reset ( vlc );
 err_reset:
	iounmap ( vlc->regs );
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
	iounmap ( vlc->regs );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** Velocity PCI device IDs */
static struct pci_device_id velocity_nics[] = {
	PCI_ROM ( 0x1106, 0x3119, "vt6122",	"VIA Velocity", 0 ),
};

/** Velocity PCI driver */
struct pci_driver velocity_driver __pci_driver = {
	.ids = velocity_nics,
	.id_count = ( sizeof ( velocity_nics ) / sizeof ( velocity_nics[0] ) ),
	.probe = velocity_probe,
	.remove = velocity_remove,
};
