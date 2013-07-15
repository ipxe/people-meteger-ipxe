#ifndef _ICP1K
#define _ICP1K

/** @file
 *
 * IC+ 1000 network driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

/** IC+ 1000 BAR size */
#define ICP1K_BAR_SIZE 256

/** IC+ 1000 Timeout in microseconds */
#define ICP1K_TIMEOUT_US	( 10 * 1000 )

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

/** A IC+ 1000 network card */
struct icp1k_nic {
	/** Registers */
	void *regs;
	/** MII interface */
	struct mii_interface mii;
};

#endif /* _ICP1K */
