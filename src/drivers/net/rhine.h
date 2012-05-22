#ifndef _RHINE_H
#define _RHINE_H

/** @file
 *
 * VIA Rhine network driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

/** Rhine BAR size */
#define RHINE_BAR_SIZE		256

/** Default timeout */
#define	RHINE_TIMEOUT_US	10000

/** Rhine descriptor format */
struct rhine_descriptor {
	uint32_t	des0;
	uint32_t	des1;
	uint32_t	buffer;
	uint32_t	next;
};

/** Rhine MAC address registers */
#define	RHINE_MAC0		0x00
#define	RHINE_MAC1		0x01
#define	RHINE_MAC2		0x02
#define	RHINE_MAC3		0x03
#define	RHINE_MAC4		0x04
#define	RHINE_MAC5		0x05

/** Receive control register */
#define	RHINE_RCR		0x06
#define	RHINE_RCR_FIFO_TRSH(_x)	(((_x) & 0x7) << 5) /*< RX FIFO threshold */
#define	RHINE_RCR_PHYS_ACCEPT	(1 << 4)	/*< Accept matching PA */
#define	RHINE_RCR_BCAST_ACCEPT	(1 << 3)	/*< Accept broadcast */
#define	RHINE_RCR_MCAST_ACCEPT	(1 << 2)	/*< Accept multicast */
#define	RHINE_RCR_RUNT_ACCEPT	(1 << 1)	/*< Accept runt frames */
#define	RHINE_RCR_ERR_ACCEPT	(1 << 0)	/*< Accept erroneous frames */

/** Transmit control register */
#define	RHINE_TCR		0x07
#define	RHINE_TCR_LOOPBACK(_x)	(((_x) & 0x3) << 1) /*< Transmit loop mode */
#define	RHINE_TCR_TAGGING	(1 << 0)	/*< 802.1P/Q packet tagging */

/** Command 0 register */
#define	RHINE_CR0		0x08
#define	RHINE_CR0_TXEN		(1 << 4)	/*< Transmit enable */
#define	RHINE_CR0_RXEN		(1 << 3)	/*< Receive enable */
#define	RHINE_CR0_STOPNIC	(1 << 2)	/*< Stop NIC */
#define	RHINE_CR0_STARTNIC	(1 << 1)	/*< Start NIC */

/** Command 1 register */
#define	RHINE_CR1		0x09
#define	RHINE_CR1_RESET		(1 << 7)	/*< Software reset */
#define	RHINE_CR1_RXPOLL	(1 << 6)	/*< Receive poll demand */
#define	RHINE_CR1_TXPOLL	(1 << 5)	/*< Xmit poll demand */
#define	RHINE_CR1_AUTOPOLL	(1 << 3)	/*< Disable autopoll */
#define	RHINE_CR1_FDX		(1 << 2)	/*< Full duplex */
#define	RIHNE_CR1_ACCUNI	(1 << 1)	/*< Disable accept unicast */

/** Transmit queue wake register */
#define	RHINE_TXQUEUE_WAKE	0x0a

/** Interrupt service 0 */
#define	RHINE_ISR0		0x0c

/** Interrupt service 1 */
#define	RHINE_ISR1		0x0d

/** Interrupt enable mask 0 */
#define	RHINE_IMR0		0x0e

/** Interrupt enable mask 1 */
#define	RHINE_IMR1		0x0f

/** RX queue descriptor base address */
#define	RHINE_RXQUEUE_BASE	0x18

/** TX queue 0 descriptor base address */
#define	RHINE_TXQUEUE_BASE	0x38

/** MII configuration */
#define	RHINE_MII_CFG		0x6c

/** MII status register */
#define	RHINE_MII_SR		0x6d
#define	RHINE_MII_SR_PHYRST	(1 << 7)	/*< PHY reset */
#define	RHINE_MII_SR_LINKNWAY	(1 << 4)	/*< Link status after N-Way */
#define	RHINE_MII_SR_PHYERR	(1 << 3)	/*< PHY device error */
#define	RHINE_MII_SR_DUPLEX	(1 << 2)	/*< Duplex mode after N-Way */
#define	RHINE_MII_SR_LINKPOLL	(1 << 1)	/*< Link status after pool */
#define	RHINE_MII_SR_LINKSPD	(1 << 0)	/*< Link speed after N-Way */

/** MII bus control 0 register */
#define	RHINE_MII_BCR0		0x6e

/** MII bus control 1 register */
#define	RHINE_MII_BCR1		0x6f

/** MII control register */
#define	RHINE_MII_CR		0x70
#define	RHINE_MII_CR_AUTOPOLL	(1 << 7)	/*< MII auto polling */
#define	RHINE_MII_CR_RDEN	(1 << 6)	/*< PHY read enable */
#define	RHINE_MII_CR_WREN	(1 << 5)	/*< PHY write enable */
#define	RHINE_MII_CR_DIRECT	(1 << 4)	/*< Direct programming mode */
#define	RHINE_MII_CR_MDIOOUT	(1 << 3)	/*< MDIO output enable */

/** MII port address */
#define	RHINE_MII_PA		0x71

/** MII read/write data */
#define	RHINE_MII_RDWR		0x72

/** EERPOM control/status register */
#define	RHINE_EEPROM_CTRL	0x74
#define	RHINE_EEPROM_CTRL_STATUS	(1 << 7) /*< EEPROM status */
#define	RHINE_EEPROM_CTRL_RELOAD	(1 << 6) /*< EEPROM reload */

/** Chip configuration A */
#define	RHINE_CHIPCFG_A		0x78

/** Chip configuration B */
#define	RHINE_CHIPCFG_B		0x79

/** Chip configuation C */
#define	RHINE_CHIPCFG_C		0x7a

/** Chip configuration D */
#define	RHINE_CHIPCFG_D		0x7b

/** A VIA Rhine network card */
struct rhine_nic {
	/** Registers */
	void *regs;
	/** MII interface */
	struct mii_interface mii;
};

#endif /* _RHINE_H */
