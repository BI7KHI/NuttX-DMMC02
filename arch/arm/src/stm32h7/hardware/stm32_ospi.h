/****************************************************************************
 * arch/arm/src/stm32h7/hardware/stm32_ospi.h
 *
 * STM32H7x3 (H723/H733/H725/H735) OCTOSPI register definitions.
 *
 * The STM32H723 has two OCTOSPI controllers (OCTOSPI1/2) but no legacy
 * QUADSPI peripheral.  This header provides the register map and bit
 * definitions for the OCTOSPI block, following the STM32H723 reference
 * manual (RM0468) and the ST CMSIS device header (stm32h723xx.h).
 *
 * Also included here are the RCC bits needed to clock OCTOSPI1/2
 * (RCC_AHB3ENR/RSTR) and to select the OCTOSPI kernel clock
 * (RCC_D1CCIPR.OSPISEL), since the stock NuttX stm32h7x3xx_rcc.h only
 * carries the legacy QUADSPI (H743) definitions.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_OSPI_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_OSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32h7/chip.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OCTOSPI register base addresses (H723: D1 AHB1 peripheral space) */

#define STM32_OCTOSPI1_BASE            0x52005000  /* OCTOSPI1 control */
#define STM32_OCTOSPI2_BASE            0x5200a000  /* OCTOSPI2 control */
#define STM32_OCTOSPIM_BASE            0x5200b400  /* OCTOSPIM IO manager */

/* OCTOSPI register offsets (STM32H723/H733/H725/H735, per RM0468 and the
 * ST CMSIS device header stm32h723xx.h).  NOTE: unlike the H743 QUADSPI,
 * the H723 OCTOSPI register map is sparse. */

#define STM32_OSPI_CR_OFFSET           0x0000  /* Control Register */
#define STM32_OSPI_DCR1_OFFSET         0x0008  /* Device Configuration Register 1 */
#define STM32_OSPI_DCR2_OFFSET         0x000c  /* Device Configuration Register 2 */
#define STM32_OSPI_DCR3_OFFSET         0x0010  /* Device Configuration Register 3 */
#define STM32_OSPI_DCR4_OFFSET         0x0014  /* Device Configuration Register 4 */
#define STM32_OSPI_SR_OFFSET           0x0020  /* Status Register */
#define STM32_OSPI_FCR_OFFSET          0x0024  /* Flag Clear Register */
#define STM32_OSPI_DLR_OFFSET          0x0040  /* Data Length Register */
#define STM32_OSPI_AR_OFFSET           0x0048  /* Address Register */
#define STM32_OSPI_DR_OFFSET           0x0050  /* Data Register */
#define STM32_OSPI_PSMKR_OFFSET        0x0080  /* Polling Status Mask Register */
#define STM32_OSPI_PSMAR_OFFSET        0x0088  /* Polling Status Match Register */
#define STM32_OSPI_PIR_OFFSET          0x0090  /* Polling Interval Register */
#define STM32_OSPI_CCR_OFFSET          0x0100  /* Communication Configuration Register */
#define STM32_OSPI_TCR_OFFSET          0x0108  /* Transfer Configuration Register */
#define STM32_OSPI_IR_OFFSET           0x0110  /* Instruction Register */
#define STM32_OSPI_ABR_OFFSET          0x0120  /* Alternate Bytes Register */
#define STM32_OSPI_LPTR_OFFSET         0x0130  /* Low-Power Timeout Register */
#define STM32_OSPI_VER_OFFSET          0x03f4  /* Version Register */
#define STM32_OSPI_ID_OFFSET           0x03f8  /* Identification Register */
#define STM32_OSPI_MID_OFFSET          0x03fc  /* Magic ID Register */

#define STM32_OSPI_CR(n)               ((n) + STM32_OSPI_CR_OFFSET)
#define STM32_OSPI_DCR1(n)             ((n) + STM32_OSPI_DCR1_OFFSET)
#define STM32_OSPI_DCR2(n)             ((n) + STM32_OSPI_DCR2_OFFSET)
#define STM32_OSPI_DCR3(n)             ((n) + STM32_OSPI_DCR3_OFFSET)
#define STM32_OSPI_DCR4(n)             ((n) + STM32_OSPI_DCR4_OFFSET)
#define STM32_OSPI_SR(n)               ((n) + STM32_OSPI_SR_OFFSET)
#define STM32_OSPI_FCR(n)              ((n) + STM32_OSPI_FCR_OFFSET)
#define STM32_OSPI_DLR(n)              ((n) + STM32_OSPI_DLR_OFFSET)
#define STM32_OSPI_AR(n)               ((n) + STM32_OSPI_AR_OFFSET)
#define STM32_OSPI_DR(n)               ((n) + STM32_OSPI_DR_OFFSET)
#define STM32_OSPI_PSMKR(n)            ((n) + STM32_OSPI_PSMKR_OFFSET)
#define STM32_OSPI_PSMAR(n)            ((n) + STM32_OSPI_PSMAR_OFFSET)
#define STM32_OSPI_PIR(n)              ((n) + STM32_OSPI_PIR_OFFSET)
#define STM32_OSPI_CCR(n)              ((n) + STM32_OSPI_CCR_OFFSET)
#define STM32_OSPI_TCR(n)              ((n) + STM32_OSPI_TCR_OFFSET)
#define STM32_OSPI_IR(n)               ((n) + STM32_OSPI_IR_OFFSET)
#define STM32_OSPI_ABR(n)              ((n) + STM32_OSPI_ABR_OFFSET)
#define STM32_OSPI_LPTR(n)             ((n) + STM32_OSPI_LPTR_OFFSET)

/* OCTOSPI_CR register bit definitions ***************************************/

#define OSPI_CR_EN                      (1 << 0)  /* Bit 0:  Enable */
#define OSPI_CR_ABORT                   (1 << 1)  /* Bit 1:  Abort request */
#define OSPI_CR_DMAEN                   (1 << 2)  /* Bit 2:  DMA Enable */
#define OSPI_CR_TCEN                    (1 << 3)  /* Bit 3:  Timeout Counter Enable */
#define OSPI_CR_DQM                     (1 << 6)  /* Bit 6:  Dual-Quad Mode */
#define OSPI_CR_FSEL                    (1 << 7)  /* Bit 7:  Flash Select */
#define OSPI_CR_FTHRES_SHIFT            (8)       /* Bits 8-12: FIFO Threshold Level */
#define OSPI_CR_FTHRES_MASK             (0x1f << OSPI_CR_FTHRES_SHIFT)
#define OSPI_CR_TEIE                    (1 << 16) /* Bit 16: Transfer Error Interrupt Enable */
#define OSPI_CR_TCIE                    (1 << 17) /* Bit 17: Transfer Complete Interrupt Enable */
#define OSPI_CR_FTIE                    (1 << 18) /* Bit 18: FIFO Threshold Interrupt Enable */
#define OSPI_CR_SMIE                    (1 << 19) /* Bit 19: Status Match Interrupt Enable */
#define OSPI_CR_TOIE                    (1 << 20) /* Bit 20: TimeOut Interrupt Enable */
#define OSPI_CR_APMS                    (1 << 22) /* Bit 22: Automatic Poll Mode Stop */
#define OSPI_CR_PMM                     (1 << 23) /* Bit 23: Polling Match Mode */
#define OSPI_CR_FMODE_SHIFT             (28)      /* Bits 28-29: Functional Mode */
#define OSPI_CR_FMODE_MASK              (3 << OSPI_CR_FMODE_SHIFT)
#  define OSPI_CR_FMODE_IND_WRITE       (0 << OSPI_CR_FMODE_SHIFT) /* Indirect write mode */
#  define OSPI_CR_FMODE_IND_READ        (1 << OSPI_CR_FMODE_SHIFT) /* Indirect read mode */
#  define OSPI_CR_FMODE_AUTO_POLL       (2 << OSPI_CR_FMODE_SHIFT) /* Automatic polling mode */
#  define OSPI_CR_FMODE_MEMMAP          (3 << OSPI_CR_FMODE_SHIFT) /* Memory-mapped mode */

/* OCTOSPI_DCR1 register bit definitions *************************************/

#define OSPI_DCR1_CKMODE                (1 << 0)  /* Bit 0:  Mode 0 / Mode 3 */
#define OSPI_DCR1_FRCK                  (1 << 1)  /* Bit 1:  Free Running Clock */
#define OSPI_DCR1_DLYBYP                (1 << 3)  /* Bit 3:  Delay Block Bypass */
#define OSPI_DCR1_CSHT_SHIFT            (8)       /* Bits 8-10: Chip Select High Time */
#define OSPI_DCR1_CSHT_MASK             (7 << OSPI_DCR1_CSHT_SHIFT)
#define OSPI_DCR1_DEVSIZE_SHIFT         (16)      /* Bits 16-20: Device Size */
#define OSPI_DCR1_DEVSIZE_MASK          (0x1f << OSPI_DCR1_DEVSIZE_SHIFT)
#define OSPI_DCR1_MTYP_SHIFT            (24)      /* Bits 24-26: Memory Type */
#define OSPI_DCR1_MTYP_MASK             (7 << OSPI_DCR1_MTYP_SHIFT)
#  define OSPI_DCR1_MTYP_MICRON         (0 << OSPI_DCR1_MTYP_SHIFT)
#  define OSPI_DCR1_MTYP_MACRONIX       (1 << OSPI_DCR1_MTYP_SHIFT)
#  define OSPI_DCR1_MTYP_APMEM          (2 << OSPI_DCR1_MTYP_SHIFT)
#  define OSPI_DCR1_MTYP_HYPERBUS       (4 << OSPI_DCR1_MTYP_SHIFT)

/* OCTOSPI_DCR2 register bit definitions *************************************/

#define OSPI_DCR2_PRESCALER_SHIFT       (0)       /* Bits 0-7: Clock prescaler */
#define OSPI_DCR2_PRESCALER_MASK        (0xff << OSPI_DCR2_PRESCALER_SHIFT)
#define OSPI_DCR2_WRAPSIZE_SHIFT        (16)      /* Bits 16-18: Wrap Size */
#define OSPI_DCR2_WRAPSIZE_MASK         (7 << OSPI_DCR2_WRAPSIZE_SHIFT)

/* OCTOSPI_DCR3 register bit definitions *************************************/

#define OSPI_DCR3_MAXTRAN_SHIFT         (0)       /* Bits 0-7: Maximum Transfer */
#define OSPI_DCR3_MAXTRAN_MASK          (0xff << OSPI_DCR3_MAXTRAN_SHIFT)
#define OSPI_DCR3_CSBOUND_SHIFT         (16)      /* Bits 16-20: CS Boundary */
#define OSPI_DCR3_CSBOUND_MASK          (0x1f << OSPI_DCR3_CSBOUND_SHIFT)

/* OCTOSPI_DCR4 register bit definitions *************************************/

#define OSPI_DCR4_REFRESH_SHIFT         (0)       /* Bits 0-31: Refresh rate */
#define OSPI_DCR4_REFRESH_MASK          (0xffffffff)

/* OCTOSPI_SR register bit definitions ***************************************/

#define OSPI_SR_TEF                     (1 << 0)  /* Bit 0: Transfer Error Flag */
#define OSPI_SR_TCF                     (1 << 1)  /* Bit 1: Transfer Complete Flag */
#define OSPI_SR_FTF                     (1 << 2)  /* Bit 2: FIFO Threshold Flag */
#define OSPI_SR_SMF                     (1 << 3)  /* Bit 3: Status Match Flag */
#define OSPI_SR_TOF                     (1 << 4)  /* Bit 4: Timeout Flag */
#define OSPI_SR_BUSY                    (1 << 5)  /* Bit 5: Busy */
#define OSPI_SR_FLEVEL_SHIFT            (8)       /* Bits 8-13: FIFO Level */
#define OSPI_SR_FLEVEL_MASK             (0x3f << OSPI_SR_FLEVEL_SHIFT)

/* OCTOSPI_FCR register bit definitions ***************************************/

#define OSPI_FCR_CTEF                   (1 << 0)  /* Bit 0: Clear Transfer Error Flag */
#define OSPI_FCR_CTCF                   (1 << 1)  /* Bit 1: Clear Transfer Complete Flag */
#define OSPI_FCR_CSMF                   (1 << 3)  /* Bit 3: Clear Status Match Flag */
#define OSPI_FCR_CTOF                   (1 << 4)  /* Bit 4: Clear Timeout Flag */
#define OSPI_FCR_ALL                    (OSPI_FCR_CTEF | OSPI_FCR_CTCF | OSPI_FCR_CSMF | OSPI_FCR_CTOF)

/* OCTOSPI_DLR register bit definitions ***************************************/

#define OSPI_DLR_DL_SHIFT               (0)       /* Bits 0-31: Data Length */
#define OSPI_DLR_DL_MASK                (0xffffffff)

/* OCTOSPI_AR register bit definitions ***************************************/

#define OSPI_AR_ADDRESS_SHIFT           (0)       /* Bits 0-31: Address */
#define OSPI_AR_ADDRESS_MASK            (0xffffffff)

/* OCTOSPI_DR register bit definitions ***************************************/

#define OSPI_DR_DATA_SHIFT              (0)       /* Bits 0-31: Data */
#define OSPI_DR_DATA_MASK               (0xffffffff)

/* OCTOSPI_CCR register bit definitions ***************************************/

#define OSPI_CCR_IMODE_SHIFT            (0)       /* Bits 0-2: Instruction Mode */
#define OSPI_CCR_IMODE_MASK             (7 << OSPI_CCR_IMODE_SHIFT)
#  define OSPI_CCR_IMODE_NONE           (0 << OSPI_CCR_IMODE_SHIFT)
#  define OSPI_CCR_IMODE_ONE            (1 << OSPI_CCR_IMODE_SHIFT)
#  define OSPI_CCR_IMODE_TWO            (2 << OSPI_CCR_IMODE_SHIFT)
#  define OSPI_CCR_IMODE_FOUR           (3 << OSPI_CCR_IMODE_SHIFT)
#  define OSPI_CCR_IMODE_EIGHT          (4 << OSPI_CCR_IMODE_SHIFT)
#define OSPI_CCR_IDTR                   (1 << 3)  /* Bit 3: Instruction Double Transfer Rate */
#define OSPI_CCR_ISIZE_SHIFT            (4)       /* Bits 4-5: Instruction Size */
#define OSPI_CCR_ISIZE_MASK             (3 << OSPI_CCR_ISIZE_SHIFT)
#  define OSPI_CCR_ISIZE_8B             (0 << OSPI_CCR_ISIZE_SHIFT)
#  define OSPI_CCR_ISIZE_16B            (1 << OSPI_CCR_ISIZE_SHIFT)
#  define OSPI_CCR_ISIZE_24B            (2 << OSPI_CCR_ISIZE_SHIFT)
#  define OSPI_CCR_ISIZE_32B            (3 << OSPI_CCR_ISIZE_SHIFT)
#define OSPI_CCR_ADMODE_SHIFT           (8)       /* Bits 8-10: Address Mode */
#define OSPI_CCR_ADMODE_MASK            (7 << OSPI_CCR_ADMODE_SHIFT)
#  define OSPI_CCR_ADMODE_NONE          (0 << OSPI_CCR_ADMODE_SHIFT)
#  define OSPI_CCR_ADMODE_ONE           (1 << OSPI_CCR_ADMODE_SHIFT)
#  define OSPI_CCR_ADMODE_TWO           (2 << OSPI_CCR_ADMODE_SHIFT)
#  define OSPI_CCR_ADMODE_FOUR          (3 << OSPI_CCR_ADMODE_SHIFT)
#  define OSPI_CCR_ADMODE_EIGHT         (4 << OSPI_CCR_ADMODE_SHIFT)
#define OSPI_CCR_ADDTR                  (1 << 11) /* Bit 11: Address Double Transfer Rate */
#define OSPI_CCR_ADSIZE_SHIFT           (12)      /* Bits 12-13: Address Size */
#define OSPI_CCR_ADSIZE_MASK            (3 << OSPI_CCR_ADSIZE_SHIFT)
#  define OSPI_CCR_ADSIZE_8B            (0 << OSPI_CCR_ADSIZE_SHIFT)
#  define OSPI_CCR_ADSIZE_16B           (1 << OSPI_CCR_ADSIZE_SHIFT)
#  define OSPI_CCR_ADSIZE_24B           (2 << OSPI_CCR_ADSIZE_SHIFT)
#  define OSPI_CCR_ADSIZE_32B           (3 << OSPI_CCR_ADSIZE_SHIFT)
#define OSPI_CCR_ABMODE_SHIFT           (16)      /* Bits 16-18: Alternate Bytes Mode */
#define OSPI_CCR_ABMODE_MASK            (7 << OSPI_CCR_ABMODE_SHIFT)
#define OSPI_CCR_ABDTR                  (1 << 19) /* Bit 19: Alternate Bytes Double Transfer Rate */
#define OSPI_CCR_ABSIZE_SHIFT           (20)      /* Bits 20-21: Alternate Bytes Size */
#define OSPI_CCR_ABSIZE_MASK            (3 << OSPI_CCR_ABSIZE_SHIFT)
#define OSPI_CCR_DMODE_SHIFT            (24)      /* Bits 24-26: Data Mode */
#define OSPI_CCR_DMODE_MASK             (7 << OSPI_CCR_DMODE_SHIFT)
#  define OSPI_CCR_DMODE_NONE           (0 << OSPI_CCR_DMODE_SHIFT)
#  define OSPI_CCR_DMODE_ONE            (1 << OSPI_CCR_DMODE_SHIFT)
#  define OSPI_CCR_DMODE_TWO            (2 << OSPI_CCR_DMODE_SHIFT)
#  define OSPI_CCR_DMODE_FOUR           (3 << OSPI_CCR_DMODE_SHIFT)
#  define OSPI_CCR_DMODE_EIGHT          (4 << OSPI_CCR_DMODE_SHIFT)
#define OSPI_CCR_DDTR                   (1 << 27) /* Bit 27: Data Double Transfer Rate */
#define OSPI_CCR_DQSE                   (1 << 29) /* Bit 29: DQS Enable */
#define OSPI_CCR_SIOO                   (1 << 31) /* Bit 31: Send Instruction Only Once */

/* OCTOSPI_TCR register bit definitions ***************************************/

#define OSPI_TCR_DCYC_SHIFT             (0)       /* Bits 0-4: Number of Dummy Cycles */
#define OSPI_TCR_DCYC_MASK              (0x1f << OSPI_TCR_DCYC_SHIFT)
#define OSPI_TCR_DHQC                   (1 << 28) /* Bit 28: Delay Hold Quarter Cycle */
#define OSPI_TCR_SSHIFT                 (1 << 30) /* Bit 30: Sample Shift */

/* OCTOSPI_IR register bit definitions ****************************************/

#define OSPI_IR_INSTRUCTION_SHIFT       (0)       /* Bits 0-31: Instruction */
#define OSPI_IR_INSTRUCTION_MASK        (0xffffffff)

/* OCTOSPI_ABR register bit definitions ***************************************/

#define OSPI_ABR_ALTERNATE_SHIFT        (0)       /* Bits 0-31: Alternate Bytes */
#define OSPI_ABR_ALTERNATE_MASK         (0xffffffff)

/* OCTOSPI_LPTR register bit definitions **************************************/

#define OSPI_LPTR_TIMEOUT_SHIFT         (0)       /* Bits 0-15: Timeout period */
#define OSPI_LPTR_TIMEOUT_MASK          (0xffff << OSPI_LPTR_TIMEOUT_SHIFT)

/* RCC bits for OCTOSPI (H723) ***********************************************/

/* RCC_AHB3ENR: OSPI1EN (bit 14), OSPI2EN (bit 19), OCTOSPIM IO manager
 * (IOMNGREN, bit 21) */

#define RCC_AHB3ENR_OSPI1EN             (1 << 14)
#define RCC_AHB3ENR_OSPI2EN             (1 << 19)
#define RCC_AHB3ENR_IOMNGREN            (1 << 21)

/* RCC_AHB3RSTR: OSPI1RST (bit 14), OSPI2RST (bit 19), IOMNGRRST (bit 21) */

#define RCC_AHB3RSTR_OSPI1RST           (1 << 14)
#define RCC_AHB3RSTR_OSPI2RST           (1 << 19)
#define RCC_AHB3RSTR_IOMNGRRST           (1 << 21)

/* RCC_D1CCIPR bits 4-5: OSPISEL - OCTOSPI kernel clock source (H723).
 * Per RM0468 and the ST HAL (RCC_OSPICLKSOURCE_*):
 *   00 = CDHCLK (D1 hclk, 240 MHz on this board)
 *   01 = PLL (pll1_q_ck)
 *   10 = PLL2 (pll2_q_ck)
 *   11 = CLKP (peripheral clock)
 */

#define RCC_D1CCIPR_OSPISEL_SHIFT       (4)
#define RCC_D1CCIPR_OSPISEL_MASK        (3 << RCC_D1CCIPR_OSPISEL_SHIFT)
#  define RCC_D1CCIPR_OSPISEL_CDHCLK    (0 << RCC_D1CCIPR_OSPISEL_SHIFT)  /* hclk (D1, 240 MHz) */
#  define RCC_D1CCIPR_OSPISEL_PLL       (1 << RCC_D1CCIPR_OSPISEL_SHIFT)  /* pll1_q_ck (120 MHz) */
#  define RCC_D1CCIPR_OSPISEL_PLL2      (2 << RCC_D1CCIPR_OSPISEL_SHIFT)  /* pll2_q_ck (96 MHz) */
#  define RCC_D1CCIPR_OSPISEL_CLKP      (3 << RCC_D1CCIPR_OSPISEL_SHIFT)  /* per_ck */
/* Compatibility alias: the default selection used by the NuttX OSPI driver */
#  define RCC_D1CCIPR_OSPISEL_D1HCLK    RCC_D1CCIPR_OSPISEL_CDHCLK

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32_OSPI_H */
