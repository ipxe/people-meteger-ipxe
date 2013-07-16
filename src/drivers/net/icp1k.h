#ifndef _ICP1K
#define _ICP1K

/** @file
 *
 * IC+ 1000 network driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

/** IC+ 1000 BAR size */
#define ICP1K_BAR_SIZE			256

/** IC+ 1000 DMA Alignment in bytes */
#define ICP1K_DMA_ALIGN			8

/** IC+ 1000 Number of transmit descriptors */
#define ICP1K_NUM_TX_DESC		8

/** Number of receive descriptors */
#define ICP1K_NUM_RX_DESC		8

/** IC+ 1000 Timeout in microseconds */
#define ICP1K_TIMEOUT_US		( 10 * 1000 )

/** IC+ 1000 low register offset */
#define ICP1K_REG_LO			0x00
/** IC+ 1000 high register offset */
#define ICP1K_REG_HI			0x04

/** IC+ 1000 transmit ring pointer register */
#define ICP1K_TX_RING			0x10

/** IC+ 1000 Asic Control Register */
#define ICP1K_ASIC_CTRL			0x30
/** IC+ 1000 Global Reset */
#define ICP1K_ASIC_CTRL_RESET		(1 << 16)
/** IC+ 1000 Receive Reset */
#define ICP1K_ASIC_CTRL_RX_RESET	(1 << 17)
/** IC+ 1000 Transmit Reset */
#define ICP1K_ASIC_CTRL_TX_RESET	(1 << 18)
/** IC+ 1000 DMA Reset */
#define ICP1K_ASIC_CTRL_DMA_RESET	(1 << 19)
/** IC+ 1000 FIFO Reset */
#define ICP1K_ASIC_CTRL_FIFO_RESET	(1 << 20)
/** IC+ 1000 Network Reset */
#define ICP1K_ASIC_CTRL_NETWORK_RESET	(1 << 21)
/** IC+ 1000 Host Reset */
#define ICP1K_ASIC_CTRL_HOST_RESET	(1 << 22)
/** IC+ 1000 Autoinit after Reset */
#define ICP1K_ASIC_CTRL_AUTOINIT	(1 << 23)
/** IC+ 1000 Reset Busy */
#define ICP1K_ASIC_CTRL_RESET_BUSY	(1 << 26)

/** IC+ 1000 Interrupt Status/Ack Register */
#define ICP1K_INT_STATUS_ACK		0x5A
/** IC+ 1000 Interrupt Link Event */
#define ICP1K_INT_LINK_EVENT		(1 << 8)

/** IC+ 1000 Interrupt Enable */
#define ICP1K_INT_ENABLE		0x5C

/** IC+ 1000 PHY Control Register */
#define ICP1K_PHY_CTRL			0x76
/** IC+ 1000 PHY Control Link state mask */
#define ICP1K_PHY_LINKSPEED_MSK		0xC0

/** Generate interrupt after packet was sent */
#define ICP1K_TX_DESC_INTR		(1 << 22)
/** Use one fragment in the descriptor */
#define ICP1K_DESC_1FRAG		(1 << 24)

#define ICP1K_TX_DESC_DONE			(1 << 31)

/** A IC+ 1000 descriptor */
struct icp1k_desc {
	/** Next descriptor */
	uint64_t next;
	/** Descriptor status */
	uint64_t status;
	/** Address + length of data */
	uint64_t data;
} __attribute__ (( packed ));

/** A IC+ 1000 ring */
struct icp1k_ring {
	/** Descriptors */
	struct icp1k_desc *desc;

	/** Consumer index */
	unsigned int cons;
	/** Producer index */
	unsigned int prod;

	/** Register */
	unsigned int reg;
	/** Number of descriptors */
	unsigned int count;
	/** Size in bytes */
	size_t size;
};

/** A IC+ 1000 network card */
struct icp1k_nic {
	/** Registers */
	void *regs;
	/** MII interface */
	struct mii_interface mii;

	/** Transmit descriptor ring */
	struct icp1k_ring tx;
	/** Receive descriptor ring */
	struct icp1k_ring rx;
};

#endif /* _ICP1K */
