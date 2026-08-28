/****************************************************************************
 * arch/arm/src/stm32h7/stm32_ospi.c
 *
 * STM32H7x3 (H723/H733/H725/H735) OCTOSPI driver.
 *
 * The STM32H723 has two OCTOSPI controllers and no legacy QUADSPI
 * peripheral.  This driver implements the NuttX QSPI interface
 * (struct qspi_dev_s, include/nuttx/spi/qspi.h) for OCTOSPI1/2 in
 * polling mode (no interrupts, no DMA), which is sufficient for
 * low-rate SPI-NOR user storage (W25Q64JV etc.).
 *
 * The board must:
 *   - configure the OCTOSPI pins (GPIO_OCTOSPI1_xxx / GPIO_OCTOSPI2_xxx)
 *     in stm32_boardinitialize(),
 *   - optionally define BOARD_OSPI_CLK to select the kernel clock source
 *     (defaults to D1HCLK, 240 MHz on this port).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_ospi.h"

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/qspi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "barriers.h"

#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_ospi.h"

#ifdef CONFIG_STM32H7_OSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Kernel clock source: the board may override with BOARD_OSPI_CLK */

#ifndef BOARD_OSPI_CLK
#  define BOARD_OSPI_CLK   RCC_D1CCIPR_OSPISEL_D1HCLK
#endif

/* Kernel clock frequency matching the source above (D1HCLK = HCLK = 240 MHz) */

#ifndef STM32_HCLK_FREQUENCY
#  error "your board.h needs to define the value of STM32_HCLK_FREQUENCY"
#endif

#ifndef OSPI_CLK_FREQUENCY
#  define OSPI_CLK_FREQUENCY  STM32_HCLK_FREQUENCY
#endif

/* Default bit rate for the flash device (W25Q64JV: 80 MHz with 4-byte
 * prescaler 3 on the 240 MHz kernel clock, matching the vendor example)
 */

#define OSPI_DEFAULT_FREQUENCY  80000000 /* vendor effective: DCR2=2 -> 240 MHz / 3 = 80 MHz */

/* FIFO threshold (words): FTF is raised when the FIFO level exceeds the
 * threshold in RX mode, or when the free FIFO space exceeds the threshold
 * in TX mode.  A threshold of 4 lets us drain/write one word per flag.
 */

#define OSPI_FTHRESHOLD         7  /* hardware threshold = FTHRES+1 = 8 bytes (vendor) */

/* Timeout for flag polling (arbitrary, generous busy-loop count) */

#define OSPI_TIMEOUT_LOOPS      20000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_ospi_dev_s
{
  struct qspi_dev_s qspi;       /* Externally visible part of the QSPI interface */
  uint32_t base;                /* OCTOSPI controller register base address */
  uint32_t frequency;           /* Requested clock frequency */
  uint32_t actual;              /* Actual clock frequency */
  uint8_t  mode;                /* QSPIDEV_MODE0 or QSPIDEV_MODE3 */
  uint8_t  nbits;               /* Width of word in bits (always 8) */
  uint8_t  intf;                /* OCTOSPI controller number (1 or 2) */
  bool     initialized;         /* TRUE: controller has been initialized */
  sem_t    exclsem;             /* Assures mutually exclusive access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t ospi_getreg(struct stm32_ospi_dev_s *priv,
                                   unsigned int offset);
static inline void ospi_putreg(struct stm32_ospi_dev_s *priv,
                               uint32_t value, unsigned int offset);
static inline void ospi_modifyreg(struct stm32_ospi_dev_s *priv,
                                  uint32_t clrbits, uint32_t setbits,
                                  unsigned int offset);
static int ospi_wait_notbusy(struct stm32_ospi_dev_s *priv);
static int ospi_wait_flag(struct stm32_ospi_dev_s *priv, uint32_t flag);
static int ospi_clear_flags(struct stm32_ospi_dev_s *priv);
static int ospi_read_fifo(struct stm32_ospi_dev_s *priv, FAR void *buffer,
                          size_t nbytes);
static int ospi_write_fifo(struct stm32_ospi_dev_s *priv,
                           FAR const void *buffer, size_t nbytes);

/* QSPI methods */

static int      ospi_lock(FAR struct qspi_dev_s *dev, bool lock);
static uint32_t ospi_setfrequency(FAR struct qspi_dev_s *dev,
                                  uint32_t frequency);
static void     ospi_setmode(FAR struct qspi_dev_s *dev,
                             enum qspi_mode_e mode);
static void     ospi_setbits(FAR struct qspi_dev_s *dev, int nbits);
static int      ospi_command(FAR struct qspi_dev_s *dev,
                             FAR struct qspi_cmdinfo_s *cmdinfo);
static int      ospi_memory(FAR struct qspi_dev_s *dev,
                            FAR struct qspi_meminfo_s *meminfo);
static FAR void *ospi_alloc(FAR struct qspi_dev_s *dev, size_t buflen);
static void     ospi_free(FAR struct qspi_dev_s *dev, FAR void *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_ospiops =
{
  ospi_lock,              /* lock */
  ospi_setfrequency,      /* setfrequency */
  ospi_setmode,           /* setmode */
  ospi_setbits,           /* setbits */
  ospi_command,           /* command */
  ospi_memory,            /* memory */
  ospi_alloc,             /* alloc */
  ospi_free               /* free */
};

static struct stm32_ospi_dev_s g_ospi1;
static struct stm32_ospi_dev_s g_ospi2;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ospi_getreg / ospi_putreg / ospi_modifyreg
 ****************************************************************************/

static inline uint32_t ospi_getreg(struct stm32_ospi_dev_s *priv,
                                   unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void ospi_putreg(struct stm32_ospi_dev_s *priv,
                               uint32_t value, unsigned int offset)
{
  putreg32(value, priv->base + offset);
}

static inline void ospi_modifyreg(struct stm32_ospi_dev_s *priv,
                                  uint32_t clrbits, uint32_t setbits,
                                  unsigned int offset)
{
  modifyreg32(priv->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: ospi_wait_notbusy
 *
 * Description:
 *   Wait until the OCTOSPI controller is idle (BUSY flag cleared).
 *
 ****************************************************************************/

static int ospi_wait_notbusy(struct stm32_ospi_dev_s *priv)
{
  volatile unsigned int timeout;

  for (timeout = 0; timeout < OSPI_TIMEOUT_LOOPS; timeout++)
    {
      if ((ospi_getreg(priv, STM32_OSPI_SR_OFFSET) & OSPI_SR_BUSY) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: ospi_wait_flag
 *
 * Description:
 *   Wait until one of the given status flags is set (or a transfer error
 *   occurs).
 *
 ****************************************************************************/

static int ospi_wait_flag(struct stm32_ospi_dev_s *priv, uint32_t flag)
{
  volatile unsigned int timeout;

  for (timeout = 0; timeout < OSPI_TIMEOUT_LOOPS; timeout++)
    {
      uint32_t sr = ospi_getreg(priv, STM32_OSPI_SR_OFFSET);

      if ((sr & OSPI_SR_TEF) != 0)
        {
          return -EIO;
        }

      if ((sr & flag) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: ospi_clear_flags
 *
 * Description:
 *   Clear all status flags before starting a new transaction.
 *
 ****************************************************************************/

static int ospi_clear_flags(struct stm32_ospi_dev_s *priv)
{
  ospi_putreg(priv, OSPI_FCR_ALL, STM32_OSPI_FCR_OFFSET);
  return OK;
}

/****************************************************************************
 * Name: ospi_read_fifo
 *
 * Description:
 *   Drain nbytes from the OCTOSPI data FIFO into the caller buffer.
 *   The indirect read transaction must already have been started (CCR
 *   written) before calling this function.
 *
 ****************************************************************************/

static int ospi_read_fifo(struct stm32_ospi_dev_s *priv, FAR void *buffer,
                          size_t nbytes)
{
  FAR uint8_t *p = (FAR uint8_t *)buffer;
  size_t remaining = nbytes;
  int ret;

  while (remaining >= 4)
    {
      uint32_t word;

      ret = ospi_wait_flag(priv, OSPI_SR_FTF | OSPI_SR_TCF);
      if (ret < 0)
        {
          return ret;
        }

      word = ospi_getreg(priv, STM32_OSPI_DR_OFFSET);
      memcpy(p, &word, 4);
      p += 4;
      remaining -= 4;
    }

  if (remaining > 0)
    {
      uint32_t word;

      /* Wait for the transfer to complete, then read the final (partial)
       * word from the FIFO.
       */

      ret = ospi_wait_flag(priv, OSPI_SR_TCF);
      if (ret < 0)
        {
          return ret;
        }

      word = ospi_getreg(priv, STM32_OSPI_DR_OFFSET);
      memcpy(p, &word, remaining);
    }

  return OK;
}

/****************************************************************************
 * Name: ospi_write_fifo
 *
 * Description:
 *   Feed nbytes into the OCTOSPI data FIFO from the caller buffer.
 *   The indirect write transaction must already have been started (CCR
 *   written) before calling this function.
 *
 ****************************************************************************/

static int ospi_write_fifo(struct stm32_ospi_dev_s *priv,
                           FAR const void *buffer, size_t nbytes)
{
  FAR const uint8_t *p = (FAR const uint8_t *)buffer;
  size_t remaining = nbytes;
  int ret;

  while (remaining >= 4)
    {
      uint32_t word;

      ret = ospi_wait_flag(priv, OSPI_SR_FTF | OSPI_SR_TCF);
      if (ret < 0)
        {
          return ret;
        }

      memcpy(&word, p, 4);
      ospi_putreg(priv, word, STM32_OSPI_DR_OFFSET);
      p += 4;
      remaining -= 4;
    }

  if (remaining > 0)
    {
      uint32_t word = 0xffffffff;

      /* Pad the final partial word: only the low `remaining` bytes are
       * transmitted (DLR limits the byte count).
       */

      ret = ospi_wait_flag(priv, OSPI_SR_FTF);
      if (ret < 0)
        {
          return ret;
        }

      memcpy(&word, p, remaining);
      ospi_putreg(priv, word, STM32_OSPI_DR_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Name: ospi_lock
 ****************************************************************************/

static int ospi_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  FAR struct stm32_ospi_dev_s *priv = (FAR struct stm32_ospi_dev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: ospi_setfrequency
 ****************************************************************************/

static uint32_t ospi_setfrequency(FAR struct qspi_dev_s *dev,
                                  uint32_t frequency)
{
  FAR struct stm32_ospi_dev_s *priv = (FAR struct stm32_ospi_dev_s *)dev;
  uint32_t presc;

  if (frequency == 0)
    {
      frequency = OSPI_DEFAULT_FREQUENCY;
    }

  priv->frequency = frequency;

  /* DCR2.PRESCALER: the actual divider is PRESCALER + 1 */

  presc = (OSPI_CLK_FREQUENCY + frequency - 1) / frequency;

  if (presc < 1)
    {
      presc = 1;
    }
  else if (presc > 256)
    {
      presc = 256;
    }

  ospi_modifyreg(priv, OSPI_DCR2_PRESCALER_MASK, presc - 1,
                 STM32_OSPI_DCR2_OFFSET);

  priv->actual = OSPI_CLK_FREQUENCY / presc;
  return priv->actual;
}

/****************************************************************************
 * Name: ospi_setmode
 *
 * Description:
 *   The OCTOSPI supports SPI clock modes 0 and 3 only (selected by
 *   DCR1.CKMODE, which controls the CLK idle level).  Modes 1 and 2 are
 *   approximated by mode 3.
 *
 ****************************************************************************/

static void ospi_setmode(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  FAR struct stm32_ospi_dev_s *priv = (FAR struct stm32_ospi_dev_s *)dev;

  priv->mode = mode;

  if (mode == QSPIDEV_MODE0)
    {
      ospi_modifyreg(priv, OSPI_DCR1_CKMODE, 0, STM32_OSPI_DCR1_OFFSET);
    }
  else
    {
      ospi_modifyreg(priv, OSPI_DCR1_CKMODE, OSPI_DCR1_CKMODE,
                     STM32_OSPI_DCR1_OFFSET);
    }
}

/****************************************************************************
 * Name: ospi_setbits
 *
 * Description:
 *   The OCTOSPI transfers are always byte oriented; only 8-bit words are
 *   supported.  The setting is stored for interface compatibility.
 *
 ****************************************************************************/

static void ospi_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
  FAR struct stm32_ospi_dev_s *priv = (FAR struct stm32_ospi_dev_s *)dev;
  priv->nbits = nbits;
}

/****************************************************************************
 * Name: ospi_command
 *
 * Description:
 *   Perform one OCTOSPI command transfer (indirect mode).
 *
 ****************************************************************************/

static int ospi_command(FAR struct qspi_dev_s *dev,
                        FAR struct qspi_cmdinfo_s *cmdinfo)
{
  FAR struct stm32_ospi_dev_s *priv = (FAR struct stm32_ospi_dev_s *)dev;
  uint8_t flags = cmdinfo->flags;
  uint32_t regval;
  uint32_t nbytes;
  uint8_t addrlen;
  uint8_t imode;
  irqstate_t irq_state;
  int ret;

  /* Instruction transfer mode: 1, 2 or 4 lines */

  imode = 1;

  if (QSPICMD_ISIQUAD(flags))
    {
      imode = 4;
    }
  else if (QSPICMD_ISIDUAL(flags))
    {
      imode = 2;
    }

  /* Wait for the controller to be idle */

  ret = ospi_wait_notbusy(priv);
  if (ret < 0)
    {
      return ret;
    }

  ospi_clear_flags(priv);

  /* Disable interrupts for the whole transaction: a preempting ISR (HRT,
   * ADC, DMA, ...) perturbs the OCTOSPI state machine mid-transfer and
   * wedges it in BUSY.  The early boot test (interrupts off) succeeds while
   * this driver (interrupts on) fails, confirming the interference.
   */

  irq_state = enter_critical_section();

  /* Program the communication registers with FMODE cleared (indirect
   * write), matching the ST HAL: the transfer is launched by re-writing AR
   * (address commands) or IR (no-address commands) AFTER the FMODE is set
   * to the final mode. */

  ospi_modifyreg(priv, OSPI_CR_FMODE_MASK, 0, STM32_OSPI_CR_OFFSET);

  /* Data length (number of bytes minus one) */

  nbytes = QSPICMD_ISDATA(flags) ? cmdinfo->buflen : 0;
  ospi_putreg(priv, nbytes > 0 ? nbytes - 1 : 0, STM32_OSPI_DLR_OFFSET);

  /* Address */

  addrlen = QSPICMD_ISADDRESS(flags) ? cmdinfo->addrlen : 0;

  if (addrlen > 0)
    {
      ospi_putreg(priv, cmdinfo->addr, STM32_OSPI_AR_OFFSET);
    }

  /* Dummy cycles: none for command transfers */

  ospi_modifyreg(priv, OSPI_TCR_DCYC_MASK, 0, STM32_OSPI_TCR_OFFSET);

  /* Communication configuration: instruction, address and data phases. */

  regval = (imode == 4) ? OSPI_CCR_IMODE_FOUR :
           (imode == 2) ? OSPI_CCR_IMODE_TWO : OSPI_CCR_IMODE_ONE;
  regval |= OSPI_CCR_ISIZE_8B;

  if (addrlen > 0)
    {
      regval |= OSPI_CCR_ADMODE_ONE;

      switch (addrlen)
        {
          case 4:
            regval |= OSPI_CCR_ADSIZE_32B;
            break;

          case 3:
            regval |= OSPI_CCR_ADSIZE_24B;
            break;

          case 2:
            regval |= OSPI_CCR_ADSIZE_16B;
            break;

          default:
            regval |= OSPI_CCR_ADSIZE_8B;
            break;
        }
    }

  if (nbytes > 0)
    {
      regval |= OSPI_CCR_DMODE_ONE;
    }

  ospi_putreg(priv, regval, STM32_OSPI_CCR_OFFSET);

  /* Instruction (configuration write) */

  ospi_putreg(priv, cmdinfo->cmd, STM32_OSPI_IR_OFFSET);

  /* Functional mode: indirect read or write */

  if (QSPICMD_ISREAD(flags))
    {
      ospi_modifyreg(priv, OSPI_CR_FMODE_MASK, OSPI_CR_FMODE_IND_READ,
                     STM32_OSPI_CR_OFFSET);
    }
  else
    {
      ospi_modifyreg(priv, OSPI_CR_FMODE_MASK, OSPI_CR_FMODE_IND_WRITE,
                     STM32_OSPI_CR_OFFSET);
    }

  /* Trigger the transfer by re-writing AR (address commands) or IR (no
   * address commands), matching the ST HAL. */

  if (addrlen > 0)
    {
      ospi_putreg(priv, cmdinfo->addr, STM32_OSPI_AR_OFFSET);
    }
  else
    {
      ospi_putreg(priv, cmdinfo->cmd, STM32_OSPI_IR_OFFSET);
    }

  /* Data phase */

  if (nbytes > 0)
    {
      if (QSPICMD_ISREAD(flags))
        {
          ret = ospi_read_fifo(priv, cmdinfo->buffer, nbytes);
        }
      else
        {
          ret = ospi_write_fifo(priv, cmdinfo->buffer, nbytes);
        }

      if (ret < 0)
        {
          leave_critical_section(irq_state);
          return ret;
        }
    }

  /* Wait for the transfer to complete and clear the flags */

  ret = ospi_wait_flag(priv, OSPI_SR_TCF);
  ospi_clear_flags(priv);

  leave_critical_section(irq_state);
  return ret;
}

/****************************************************************************
 * Name: ospi_memory
 *
 * Description:
 *   Perform one OCTOSPI memory transfer (indirect mode).
 *
 ****************************************************************************/

static int ospi_memory(FAR struct qspi_dev_s *dev,
                       FAR struct qspi_meminfo_s *meminfo)
{
  FAR struct stm32_ospi_dev_s *priv = (FAR struct stm32_ospi_dev_s *)dev;
  uint8_t flags = meminfo->flags;
  bool read = QSPIMEM_ISREAD(flags);
  uint32_t regval;
  uint8_t imode;
  uint8_t amode;
  uint8_t dmode;
  irqstate_t irq_state;
  int ret;

  /* Instruction transfer mode: 1, 2 or 4 lines */

  imode = 1;

  if (QSPIMEM_ISIQUAD(flags))
    {
      imode = 4;
    }
  else if (QSPIMEM_ISIDUAL(flags))
    {
      imode = 2;
    }

  /* Address and data modes: dual/quad I/O for reads, 1-line for writes */

  amode = 1;
  dmode = 1;

  if (read)
    {
      if (QSPIMEM_ISQUADIO(flags))
        {
          amode = 4;
          dmode = 4;
        }
      else if (QSPIMEM_ISDUALIO(flags))
        {
          amode = 2;
          dmode = 2;
        }
    }

  /* Wait for the controller to be idle */

  ret = ospi_wait_notbusy(priv);
  if (ret < 0)
    {
      return ret;
    }

  ospi_clear_flags(priv);

  /* Disable interrupts for the whole transaction: a preempting ISR (HRT,
   * ADC, DMA, ...) perturbs the OCTOSPI state machine mid-transfer and
   * wedges it in BUSY.  The early boot test (interrupts off) succeeds while
   * this driver (interrupts on) fails, confirming the interference.
   */

  irq_state = enter_critical_section();

  /* Program the communication registers with FMODE cleared (indirect
   * write), matching the ST HAL: the transfer is launched by re-writing AR
   * (memory transfers always carry an address) AFTER the FMODE is set. */

  ospi_modifyreg(priv, OSPI_CR_FMODE_MASK, 0, STM32_OSPI_CR_OFFSET);

  /* Data length (number of bytes minus one) */

  ospi_putreg(priv, meminfo->buflen - 1, STM32_OSPI_DLR_OFFSET);

  /* Address */

  ospi_putreg(priv, meminfo->addr, STM32_OSPI_AR_OFFSET);

  /* Dummy cycles (reads only) */

  ospi_modifyreg(priv, OSPI_TCR_DCYC_MASK,
                 read ? (meminfo->dummies << OSPI_TCR_DCYC_SHIFT) : 0,
                 STM32_OSPI_TCR_OFFSET);

  /* Communication configuration: instruction, address and data phases. */

  regval = (imode == 4) ? OSPI_CCR_IMODE_FOUR :
           (imode == 2) ? OSPI_CCR_IMODE_TWO : OSPI_CCR_IMODE_ONE;
  regval |= OSPI_CCR_ISIZE_8B;

  regval |= (amode == 4) ? OSPI_CCR_ADMODE_FOUR :
            (amode == 2) ? OSPI_CCR_ADMODE_TWO : OSPI_CCR_ADMODE_ONE;

  switch (meminfo->addrlen)
    {
      case 4:
        regval |= OSPI_CCR_ADSIZE_32B;
        break;

      case 3:
        regval |= OSPI_CCR_ADSIZE_24B;
        break;

      case 2:
        regval |= OSPI_CCR_ADSIZE_16B;
        break;

      default:
        regval |= OSPI_CCR_ADSIZE_8B;
        break;
    }

  regval |= (dmode == 4) ? OSPI_CCR_DMODE_FOUR :
            (dmode == 2) ? OSPI_CCR_DMODE_TWO : OSPI_CCR_DMODE_ONE;

  ospi_putreg(priv, regval, STM32_OSPI_CCR_OFFSET);

  /* Instruction (configuration write) */

  ospi_putreg(priv, meminfo->cmd, STM32_OSPI_IR_OFFSET);

  /* Functional mode: indirect read or write */

  ospi_modifyreg(priv, OSPI_CR_FMODE_MASK,
                 read ? OSPI_CR_FMODE_IND_READ : OSPI_CR_FMODE_IND_WRITE,
                 STM32_OSPI_CR_OFFSET);

  /* Trigger the transfer by re-writing AR (matches the ST HAL) */

  ospi_putreg(priv, meminfo->addr, STM32_OSPI_AR_OFFSET);

  /* Data phase */

  if (read)
    {
      ret = ospi_read_fifo(priv, meminfo->buffer, meminfo->buflen);
    }
  else
    {
      ret = ospi_write_fifo(priv, meminfo->buffer, meminfo->buflen);
    }

  if (ret < 0)
    {
      leave_critical_section(irq_state);
      return ret;
    }

  /* Wait for the transfer to complete and clear the flags */

  ret = ospi_wait_flag(priv, OSPI_SR_TCF);
  ospi_clear_flags(priv);

  leave_critical_section(irq_state);
  return ret;
}

/****************************************************************************
 * Name: ospi_alloc / ospi_free
 ****************************************************************************/

static FAR void *ospi_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(buflen);
}

static void ospi_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  kmm_free(buffer);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ospi_initialize
 *
 * Description:
 *   Initialize the selected OCTOSPI port (1 or 2).
 *
 ****************************************************************************/

FAR struct qspi_dev_s *stm32_ospi_initialize(int intf)
{
  FAR struct stm32_ospi_dev_s *priv;
  uint32_t enbit;
  uint32_t rstbit;
  uint32_t regval;

  if (intf == 1)
    {
      priv = &g_ospi1;
      enbit = RCC_AHB3ENR_OSPI1EN;
      rstbit = RCC_AHB3RSTR_OSPI1RST;
    }
  else if (intf == 2)
    {
      priv = &g_ospi2;
      enbit = RCC_AHB3ENR_OSPI2EN;
      rstbit = RCC_AHB3RSTR_OSPI2RST;
    }
  else
    {
      return NULL;
    }

  if (!priv->initialized)
    {
      /* Enable the peripheral clock and apply a reset pulse */

      modifyreg32(STM32_RCC_AHB3ENR, 0, enbit | RCC_AHB3ENR_IOMNGREN);

      /* Reset the OCTOSPIM (IO manager) together with the controller: the
       * bootloader may leave the IO manager in a state that never ACKs the
       * OCTOSPI pad request, which hangs every transfer in BUSY. */

      modifyreg32(STM32_RCC_AHB3RSTR, 0, rstbit | RCC_AHB3RSTR_IOMNGRRST);
      modifyreg32(STM32_RCC_AHB3RSTR, rstbit | RCC_AHB3RSTR_IOMNGRRST, 0);

      /* Select the OCTOSPI kernel clock source */

      modifyreg32(STM32_RCC_D1CCIPR, RCC_D1CCIPR_OSPISEL_MASK,
                  BOARD_OSPI_CLK);

      memset(priv, 0, sizeof(*priv));

      priv->qspi.ops   = &g_ospiops;
      priv->base       = (intf == 1) ? STM32_OCTOSPI1_BASE : STM32_OCTOSPI2_BASE;
      priv->intf       = intf;
      priv->mode       = QSPIDEV_MODE3;
      priv->nbits      = 8;
      priv->frequency  = OSPI_DEFAULT_FREQUENCY;
      priv->actual     = OSPI_DEFAULT_FREQUENCY;
      priv->initialized = true;

      nxsem_init(&priv->exclsem, 0, 1);

      /* Device configuration 1:
       *   CKMODE  = mode 3 (CLK high while nCS high)
       *   CSHT    = 1 cycle
       *   DLYBYP  = 1 (delay block bypassed)
       *   DEVSIZE = 22 (hardware field is log2(size)-1: 2^23 = 8 MB,
       *                    W25Q64; matches the vendor HAL write of
       *                    DeviceSize-1 with DeviceSize = 23)
       *   MTYP    = Micron (standard SPI NOR)
       */

      regval  = OSPI_DCR1_CKMODE;
      regval |= (0 << OSPI_DCR1_CSHT_SHIFT); /* vendor: CSHT raw 0 (=1 cycle) */
      regval |= OSPI_DCR1_DLYBYP;
      regval |= (22 << OSPI_DCR1_DEVSIZE_SHIFT);
      regval |= OSPI_DCR1_MTYP_MICRON;
      ospi_putreg(priv, regval, STM32_OSPI_DCR1_OFFSET);

      /* Device configuration 2: clock prescaler for the default rate */

      (void)ospi_setfrequency(&priv->qspi, OSPI_DEFAULT_FREQUENCY);

      /* Control register: enable, FIFO threshold */

      regval  = OSPI_CR_EN;
      regval |= (OSPI_FTHRESHOLD << OSPI_CR_FTHRES_SHIFT);
      ospi_putreg(priv, regval, STM32_OSPI_CR_OFFSET);

      /* Transfer configuration: sample shift (half cycle), as used by the
       * vendor example for 80 MHz operation.
       */

      regval = OSPI_TCR_SSHIFT;
      ospi_putreg(priv, regval, STM32_OSPI_TCR_OFFSET);
    }

  return &priv->qspi;
}

#endif /* CONFIG_STM32H7_OSPI */
