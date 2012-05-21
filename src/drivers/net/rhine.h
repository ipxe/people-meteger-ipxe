#ifndef _RHINE_H
#define _RHINE_H

/** @file
 *
 * VIA Rhine network driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

/** Skeleton BAR size */
#define RHINE_BAR_SIZE 256

/** Rhine descriptor format */
struct rhine_descriptor {
	uint32_t	des0;
	uint32_t	des1;
	uint32_t	buffer;
	uint32_t	next;
};

/** Rhine internal registers */
#define	RHINE_MAC0		0x00
#define	RHINE_MAC1		0x01
#define	RHINE_MAC2		0x02
#define	RHINE_MAC3		0x03
#define	RHINE_MAC4		0x04
#define	RHINE_MAC5		0x05
#define	RHINE_RCR		0x06
#define	RHINE_RCR_FIFO_TRSH(_x)	(((_x) & 0x7) << 5)
#define	RHINE_RCR_PHYS_ACCEPT	(1 << 4)
#define	RHINE_RCR_BCAST_ACCEPT	(1 << 3)
#define	RHINE_RCR_MCAST_ACCEPT	(1 << 2)
#define	RHINE_RCR_RUNT_ACCEPT	(1 << 1)
#define	RHINE_RCR_ERR_ACCEPT	(1 << 0)
#define	RHINE_TCR		0x07
#define	RHINE_TCR_LOOPBACK	(((_x) & 0x3) << 1)
#define	RHINE_TCR_TAGGING	(1 << 0)
#define	RHINE_CR0		0x08
#define	RHINE_CR0_TXEN		(1 << 4)
#define	RHINE_CR0_RXEN		(1 << 3)
#define	RHINE_CR0_STOPNIC	(1 << 2)
#define	RHINE_CR0_STARTNIC	(1 << 1)
#define	RHINE_CR1		0x09
#define	RHINE_CR1_RESET		(1 << 7)
#define	RHINE_CR1_RXPOLL	(1 << 6)
#define	RHINE_CR1_TXPOLL	(1 << 5)
#define	RHINE_CR1_AUTOPOLL	(1 << 3)
#define	RHINE_CR1_FDX		(1 << 2)
#define	RIHNE_CR1_ACCUNI	(1 << 1)
#define	RHINE_TXQUEUE_WAKE	0x0a
#define	RHINE_ISR0		0x0c
#define	RHINE_ISR1		0x0d
#define	RHINE_IMR0		0x0e
#define	RHINE_IMR1		0x0f
#define	RHINE_RXQUEUE_BASE	0x18
#define	RHINE_TXQUEUE_BASE	0x38

/** Rhine MII interface */
#define	RHINE_MII_CFG		0x6c
#define	RHINE_MII_SR		0x6d
#define	RHINE_MII_SR_PHYRST	(1 << 7)
#define	RHINE_MII_SR_LINKNWAY	(1 << 4)
#define	RHINE_MII_SR_PHYERR	(1 << 3)
#define	RHINE_MII_SR_DUPLEX	(1 << 2)
#define	RHINE_MII_SR_LINKPOLL	(1 << 1)
#define	RHINE_MII_SR_LINKSPD	(1 << 0)
#define	RHINE_MII_BCR0		0x6e
#define	RHINE_MII_BCR1		0x6f
#define	RHINE_MII_CR		0x70
#define	RHINE_MII_CR_AUTOPOLL	(1 << 7)
#define	RHINE_MII_CR_RDEN	(1 << 6)
#define	RHINE_MII_CR_WREN	(1 << 5)
#define	RHINE_MII_CR_DIRECT	(1 << 4)
#define	RHINE_MII_CR_MDIOOUT	(1 << 3)
#define	RHINE_MII_PA		0x71
#define	RHINE_MII_RDWR		0x72

/** EERPOM interface */
#define	RHINE_EEPROM_CTRL	0x74
#define	RHINE_EEPROM_CTRL_STATUS	(1 << 7)
#define	RHINE_EEPROM_CTRL_RELOAD	(1 << 6)
#define	RHINE_CHIPCFG_A		0x78
#define	RHINE_CHIPCFG_B		0x79
#define	RHINE_CHIPCFG_C		0x7a
#define	RHINE_CHIPCFG_D		0x7b

/** A skeleton network card */
struct rhine_nic {
	/** Registers */
	void *regs;
	/** MII interface */
	struct mii_interface mii;
};

#endif /* _RHINE_H */
