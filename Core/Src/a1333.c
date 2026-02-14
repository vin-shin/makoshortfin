/**
 * @file a1333.c
 * @brief Allegro A1333 Hall-Effect Angle Sensor SPI Driver Implementation
 *
 * Key protocol notes:
 *   - SPI Mode 3 (CPOL=1, CPHA=1), 16-bit frames
 *   - Frame format: [0 | R/W | A5:A0 | D7:D0]
 *     Bit 15 = 0 (always), Bit 14 = R/W (0=read, 1=write)
 *   - Reads are PIPELINED: send read command in frame N, data arrives in frame N+1
 *   - For a full 16-bit register read, use an even address
 *   - MOSI sampled on rising SCLK edge, MISO shifts out on falling SCLK edge
 */

#include "a1333.h"
#include <string.h>

/* ── Internal Helpers ──────────────────────────────────────────────── */

/**
 * @brief Assert chip select (active low)
 */
static inline void cs_low(A1333_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Deassert chip select
 */
static inline void cs_high(A1333_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Small delay to meet tCS_IDLE (200 ns min between frames)
 *        At 168 MHz, a few NOPs or a short loop is sufficient.
 */
static inline void cs_idle_delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < 4; i++) { __NOP(); }
}

/**
 * @brief Build a 16-bit SPI write command
 * @param addr 6-bit register address (use even address for full 16-bit reg)
 * @param data 8-bit data byte
 * @return 16-bit SPI MOSI word
 *
 * Format: [0 | 1 | A5 A4 A3 A2 A1 A0 | D7 D6 D5 D4 D3 D2 D1 D0]
 */
static uint16_t build_write_cmd(uint8_t addr, uint8_t data)
{
    uint16_t cmd = 0;
    cmd |= (1U << 14);              // R/W = 1 (write)
    cmd |= ((uint16_t)(addr & 0x3F) << 8);  // 6-bit address
    cmd |= data;                     // 8-bit data
    return cmd;
}

/**
 * @brief Build a 16-bit SPI read command
 * @param addr 6-bit register address
 * @return 16-bit SPI MOSI word
 *
 * Format: [0 | 0 | A5 A4 A3 A2 A1 A0 | X X X X X X X X]
 */
static uint16_t build_read_cmd(uint8_t addr)
{
    uint16_t cmd = 0;
    cmd |= ((uint16_t)(addr & 0x3F) << 8);  // 6-bit address, R/W = 0
    return cmd;
}

/**
 * @brief Execute a single 16-bit SPI transaction (full duplex)
 * @param dev    Device handle
 * @param tx     16-bit word to transmit on MOSI
 * @param rx     Pointer to store 16-bit word received on MISO (can be NULL)
 * @return A1333_OK or A1333_ERR_SPI
 *
 * NOTE: STM32 HAL SPI in 16-bit mode sends MSB first by default.
 *       Make sure your SPI peripheral is configured for 16-bit data size.
 */
static A1333_Status spi_transfer_16(A1333_Handle *dev, uint16_t tx, uint16_t *rx)
{
    uint16_t rx_buf = 0;
    HAL_StatusTypeDef hal_status;

    cs_low(dev);

    hal_status = HAL_SPI_TransmitReceive(dev->hspi,
                                         (uint8_t *)&tx,
                                         (uint8_t *)&rx_buf,
                                         1,       /* 1 x 16-bit unit */
                                         2);      /* 2 ms timeout */

    cs_high(dev);
    cs_idle_delay();  /* tCS_IDLE >= 200 ns */

    if (hal_status != HAL_OK)
        return A1333_ERR_SPI;

    if (rx != NULL)
        *rx = rx_buf;

    return A1333_OK;
}

/* ── Parity Check ──────────────────────────────────────────────────── */

/**
 * @brief Check odd parity of a 16-bit word
 *        A1333 angle registers use odd parity across bits [14:0]
 */
static bool check_odd_parity(uint16_t word)
{
    /* Count number of 1s in bits [14:0] */
    uint16_t v = word & 0x7FFF;
    uint8_t count = 0;
    while (v) {
        count += (v & 1);
        v >>= 1;
    }
    return (count & 1) == 1;  /* Odd parity: should be odd number of 1s */
}

/* ══════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ══════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the A1333 driver
 *
 * IMPORTANT: Configure your SPI peripheral BEFORE calling this:
 *   - Mode 3: CPOL=HIGH, CPHA=2EDGE
 *   - Data size: 16-bit
 *   - MSB first
 *   - NSS: Software (we manage CS manually via GPIO)
 *   - Clock <= 10 MHz
 *   - Full duplex master
 */
A1333_Status A1333_Init(A1333_Handle *dev, SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;

    /* Ensure CS starts high (deselected) */
    cs_high(dev);
    HAL_Delay(2);  /* Allow sensor to finish boot (~1.5 ms typical) */

    /* Send a NOP to flush the interface */
    uint16_t nop = build_read_cmd(A1333_REG_NOP);
    A1333_Status s = spi_transfer_16(dev, nop, NULL);
    if (s != A1333_OK) return s;

    /* Verify communication: read status register, check RIDC = 0x8 */
    uint16_t sta;
    s = A1333_ReadReg(dev, A1333_REG_STA, &sta);
    if (s != A1333_OK) return s;

    uint8_t ridc = (sta >> 12) & 0xF;
    if (ridc != 0x8)
        return A1333_ERR_SPI;  /* Unexpected response — wiring/config issue */

    return A1333_OK;
}

/* ── Register Read/Write ───────────────────────────────────────────── */

/**
 * @brief Read a 16-bit register
 *
 * Two-frame pipelined read:
 *   Frame 1: Send read command → MISO data is stale (ignore)
 *   Frame 2: Send NOP         → MISO returns requested register contents
 *
 * @param addr Even address reads full 16-bit register.
 *             Odd address reads only the LSB (MSB = 0x00).
 */
A1333_Status A1333_ReadReg(A1333_Handle *dev, uint8_t addr, uint16_t *data)
{
    A1333_Status s;
    uint16_t rx;

    /* Frame 1: issue the read command */
    uint16_t cmd = build_read_cmd(addr);
    s = spi_transfer_16(dev, cmd, &rx);  /* rx is from previous (stale) */
    if (s != A1333_OK) return s;

    /* Frame 2: clock out the response with a NOP */
    uint16_t nop = build_read_cmd(A1333_REG_NOP);
    s = spi_transfer_16(dev, nop, &rx);
    if (s != A1333_OK) return s;

    *data = rx;
    return A1333_OK;
}

/**
 * @brief Write an 8-bit value to a register byte
 *
 * To write a full 16-bit register, call this twice (even addr then odd addr).
 *
 * @param addr 6-bit address of the byte to write
 * @param data 8-bit data value
 */
A1333_Status A1333_WriteReg(A1333_Handle *dev, uint8_t addr, uint8_t data)
{
    uint16_t cmd = build_write_cmd(addr, data);
    return spi_transfer_16(dev, cmd, NULL);
}

/* ── Angle Reading ─────────────────────────────────────────────────── */

A1333_Status A1333_ReadAngle12(A1333_Handle *dev, uint16_t *raw, float *degrees)
{
    uint16_t reg;
    A1333_Status s = A1333_ReadReg(dev, A1333_REG_ANG, &reg);
    if (s != A1333_OK) return s;

    /* Optional: verify odd parity */
    if (!check_odd_parity(reg))
        return A1333_ERR_PARITY;

    uint16_t angle_raw = reg & A1333_ANG_ANGLE_MASK;

    if (raw != NULL)
        *raw = angle_raw;
    if (degrees != NULL)
        *degrees = (float)angle_raw * (360.0f / 4096.0f);

    /* Check error/undervoltage flags embedded in the angle register */
    if (reg & A1333_ANG_EF_MASK)
        return A1333_ERR_DEVICE;

    return A1333_OK;
}

A1333_Status A1333_ReadAngle15(A1333_Handle *dev, uint16_t *raw, float *degrees)
{
    uint16_t reg;
    A1333_Status s = A1333_ReadReg(dev, A1333_REG_ANG15, &reg);
    if (s != A1333_OK) return s;

    uint16_t angle_raw = reg & 0x7FFF;

    if (raw != NULL)
        *raw = angle_raw;
    if (degrees != NULL)
        *degrees = (float)angle_raw * (360.0f / 32768.0f);

    return A1333_OK;
}

A1333_Status A1333_ReadAngleHysteresis(A1333_Handle *dev, uint16_t *raw, float *degrees)
{
    uint16_t reg;
    A1333_Status s = A1333_ReadReg(dev, A1333_REG_HANG, &reg);
    if (s != A1333_OK) return s;

    if (!check_odd_parity(reg))
        return A1333_ERR_PARITY;

    uint16_t angle_raw = reg & A1333_ANG_ANGLE_MASK;

    if (raw != NULL)
        *raw = angle_raw;
    if (degrees != NULL)
        *degrees = (float)angle_raw * (360.0f / 4096.0f);

    if (reg & A1333_ANG_EF_MASK)
        return A1333_ERR_DEVICE;

    return A1333_OK;
}

/* ── Diagnostics ───────────────────────────────────────────────────── */

A1333_Status A1333_ReadStatus(A1333_Handle *dev, uint16_t *status)
{
    return A1333_ReadReg(dev, A1333_REG_STA, status);
}

A1333_Status A1333_ReadErrors(A1333_Handle *dev, uint16_t *errors)
{
    return A1333_ReadReg(dev, A1333_REG_ERR, errors);
}

A1333_Status A1333_ReadWarnings(A1333_Handle *dev, uint16_t *warnings)
{
    return A1333_ReadReg(dev, A1333_REG_WARN, warnings);
}

A1333_Status A1333_ClearErrors(A1333_Handle *dev)
{
    /* Write CLE bit (bit 8) in CTRL register upper byte (addr 0x1E) */
    /* CTRL[15:8] = SPECIAL[3:0] | 0 | CLS | CLW | CLE */
    /* CLE = bit 8, so upper byte bit 0 */
    return A1333_WriteReg(dev, A1333_REG_CTRL, 0x01);  /* CLE = 1 */
}

A1333_Status A1333_ClearWarnings(A1333_Handle *dev)
{
    /* CLW = bit 9 in CTRL, which is bit 1 of upper byte */
    return A1333_WriteReg(dev, A1333_REG_CTRL, 0x02);  /* CLW = 1 */
}

A1333_Status A1333_IsAngleValid(A1333_Handle *dev, bool *valid)
{
    uint16_t sta;
    A1333_Status s = A1333_ReadStatus(dev, &sta);
    if (s != A1333_OK) return s;

    *valid = (sta & A1333_STA_AOK) != 0;
    return A1333_OK;
}

/* ── Temperature & Field ───────────────────────────────────────────── */

A1333_Status A1333_ReadTemperature(A1333_Handle *dev, float *temp_c)
{
    uint16_t reg;
    A1333_Status s = A1333_ReadReg(dev, A1333_REG_TSEN, &reg);
    if (s != A1333_OK) return s;

    /* TEMPERATURE is bits [11:0], signed 12-bit (2's complement) */
    int16_t raw = (int16_t)(reg & 0x0FFF);
    if (raw & 0x0800)           /* Sign-extend 12-bit to 16-bit */
        raw |= (int16_t)0xF000;

    /* Value is in 1/8 degree relative to 25°C */
    *temp_c = ((float)raw / 8.0f) + 25.0f;
    return A1333_OK;
}

A1333_Status A1333_ReadFieldStrength(A1333_Handle *dev, uint16_t *gauss)
{
    uint16_t reg;
    A1333_Status s = A1333_ReadReg(dev, A1333_REG_FIELD, &reg);
    if (s != A1333_OK) return s;

    *gauss = reg & 0x0FFF;  /* 12-bit unsigned */
    return A1333_OK;
}

/* ── Turns Counter ─────────────────────────────────────────────────── */

A1333_Status A1333_ReadTurns(A1333_Handle *dev, int16_t *turns)
{
    uint16_t reg;
    A1333_Status s = A1333_ReadReg(dev, A1333_REG_TURNS, &reg);
    if (s != A1333_OK) return s;

    /* TURNS is bits [11:0], signed 12-bit (2's complement) */
    int16_t raw = (int16_t)(reg & 0x0FFF);
    if (raw & 0x0800)
        raw |= (int16_t)0xF000;

    *turns = raw;
    return A1333_OK;
}

A1333_Status A1333_ResetTurns(A1333_Handle *dev)
{
    /* SPECIAL = 0x4 (turns-counter reset), then INITIATE_SPECIAL = 0x46 */
    A1333_Status s;

    /* Write SPECIAL (bits [15:12] of CTRL) = 0100b → upper byte = 0x40 */
    s = A1333_WriteReg(dev, A1333_REG_CTRL, 0x40);
    if (s != A1333_OK) return s;

    /* Write INITIATE_SPECIAL (bits [7:0] of CTRL) = 0x46 */
    s = A1333_WriteReg(dev, A1333_REG_CTRL + 1, 0x46);
    if (s != A1333_OK) return s;

    HAL_Delay(1);  /* Allow command to execute */
    return A1333_OK;
}

/* ── EEPROM Unlock ─────────────────────────────────────────────────── */

A1333_Status A1333_Unlock(A1333_Handle *dev)
{
    A1333_Status s;
    const uint8_t unlock_seq[] = { 0x00, 0x27, 0x81, 0x1F, 0x77 };

    for (int i = 0; i < 5; i++) {
        s = A1333_WriteReg(dev, A1333_REG_IKEY, unlock_seq[i]);
        if (s != A1333_OK) return s;
    }

    /* Verify unlock by checking CUL bit (bit 0 of address 0x3D) */
    uint16_t key_reg;
    s = A1333_ReadReg(dev, A1333_REG_IKEY, &key_reg);
    if (s != A1333_OK) return s;

    if (!(key_reg & 0x01))
        return A1333_ERR_UNLOCK_FAIL;

    return A1333_OK;
}

/* ── Extended Read (EEPROM / Shadow) ───────────────────────────────── */

/**
 * @brief Internal: read from an extended address (EEPROM or shadow)
 */
static A1333_Status extended_read(A1333_Handle *dev, uint8_t ext_addr, uint32_t *data)
{
    A1333_Status s;

    /* Step 1: Write extended address to ERA lower byte (register 0x0B) */
    s = A1333_WriteReg(dev, A1333_REG_ERA + 1, ext_addr);
    if (s != A1333_OK) return s;

    /* Step 2: Initiate read — write EXR bit (bit 15 of ERCS = 0x0C) */
    s = A1333_WriteReg(dev, A1333_REG_ERCS, 0x80);  /* EXR = 1 */
    if (s != A1333_OK) return s;

    /* Step 3: Poll RDN (bit 0 of ERCS+1 = address 0x0D) */
    uint16_t ercs;
    uint32_t timeout = 100;
    do {
        s = A1333_ReadReg(dev, A1333_REG_ERCS, &ercs);
        if (s != A1333_OK) return s;
        if (ercs & 0x0001) break;  /* RDN set */
        HAL_Delay(1);
    } while (--timeout);

    if (timeout == 0)
        return A1333_ERR_TIMEOUT;

    /* Step 4: Read ERDH (upper 16 bits) and ERDL (lower 16 bits) */
    uint16_t hi, lo;
    s = A1333_ReadReg(dev, A1333_REG_ERDH, &hi);
    if (s != A1333_OK) return s;
    s = A1333_ReadReg(dev, A1333_REG_ERDL, &lo);
    if (s != A1333_OK) return s;

    *data = ((uint32_t)hi << 16) | lo;
    return A1333_OK;
}

A1333_Status A1333_ReadEEPROM(A1333_Handle *dev, uint8_t eep_addr, uint32_t *data)
{
    return extended_read(dev, eep_addr, data);
}

A1333_Status A1333_ReadShadow(A1333_Handle *dev, uint8_t eep_addr, uint32_t *data)
{
    return extended_read(dev, eep_addr + A1333_SHADOW_OFFSET, data);
}

/* ── Extended Write (EEPROM / Shadow) ──────────────────────────────── */

/**
 * @brief Internal: write to an extended address (EEPROM or shadow)
 *
 * EEPROM writes take ~24 ms. Shadow writes are immediate.
 */
static A1333_Status extended_write(A1333_Handle *dev, uint8_t ext_addr,
                                   uint32_t data, bool is_eeprom)
{
    A1333_Status s;

    /* Step 1: Write address to EWA lower byte (register 0x03) */
    s = A1333_WriteReg(dev, A1333_REG_EWA + 1, ext_addr);
    if (s != A1333_OK) return s;

    /* Step 2: Write 32-bit data across EWD registers */
    s = A1333_WriteReg(dev, A1333_REG_EWDH,     (data >> 24) & 0xFF);
    if (s != A1333_OK) return s;
    s = A1333_WriteReg(dev, A1333_REG_EWDH + 1,  (data >> 16) & 0xFF);
    if (s != A1333_OK) return s;
    s = A1333_WriteReg(dev, A1333_REG_EWDL,     (data >> 8)  & 0xFF);
    if (s != A1333_OK) return s;
    s = A1333_WriteReg(dev, A1333_REG_EWDL + 1,  data & 0xFF);
    if (s != A1333_OK) return s;

    /* Step 3: Initiate write — set EXW bit (bit 15 of EWCS = 0x08) */
    s = A1333_WriteReg(dev, A1333_REG_EWCS, 0x80);
    if (s != A1333_OK) return s;

    /* Step 4: Poll WDN (bit 0 of EWCS+1 = address 0x09) */
    uint16_t ewcs;
    uint32_t timeout = is_eeprom ? 50 : 10;  /* EEPROM: up to 24 ms */
    do {
        HAL_Delay(1);
        s = A1333_ReadReg(dev, A1333_REG_EWCS, &ewcs);
        if (s != A1333_OK) return s;
        if (ewcs & 0x0001) break;  /* WDN set */
    } while (--timeout);

    if (timeout == 0)
        return A1333_ERR_TIMEOUT;

    return A1333_OK;
}

A1333_Status A1333_WriteEEPROM(A1333_Handle *dev, uint8_t eep_addr, uint32_t data)
{
    return extended_write(dev, eep_addr, data, true);
}

A1333_Status A1333_WriteShadow(A1333_Handle *dev, uint8_t eep_addr, uint32_t data)
{
    return extended_write(dev, eep_addr + A1333_SHADOW_OFFSET, data, false);
}

/* ── Configuration Helpers ─────────────────────────────────────────── */

A1333_Status A1333_SetZeroOffset(A1333_Handle *dev, uint16_t offset_12bit)
{
    /* Read current ANG EEPROM (0x1C), modify ZERO_OFFSET [11:0], write to shadow */
    uint32_t ang_val;
    A1333_Status s = A1333_ReadShadow(dev, A1333_EEP_ANG, &ang_val);
    if (s != A1333_OK) return s;

    /* Clear bits [11:0], insert new offset */
    ang_val &= ~0x000FFF;
    ang_val |= (offset_12bit & 0xFFF);

    return A1333_WriteShadow(dev, A1333_EEP_ANG, ang_val);
}

A1333_Status A1333_SetRotationDirection(A1333_Handle *dev, bool counter_clockwise)
{
    uint32_t ang_val;
    A1333_Status s = A1333_ReadShadow(dev, A1333_EEP_ANG, &ang_val);
    if (s != A1333_OK) return s;

    /* RO is bit 18 */
    if (counter_clockwise)
        ang_val |= (1UL << 18);
    else
        ang_val &= ~(1UL << 18);

    return A1333_WriteShadow(dev, A1333_EEP_ANG, ang_val);
}

/* ── CRC Calculation (for 20-bit SPI mode) ─────────────────────────── */

uint8_t A1333_CalculateCRC(uint16_t input)
{
    bool CRC0 = true, CRC1 = true, CRC2 = true, CRC3 = true;
    bool DoInvert;
    uint16_t mask = 0x8000;

    for (int i = 0; i < 16; i++) {
        DoInvert = ((input & mask) != 0) ^ CRC3;
        CRC3 = CRC2;
        CRC2 = CRC1;
        CRC1 = CRC0 ^ DoInvert;
        CRC0 = DoInvert;
        mask >>= 1;
    }

    return (uint8_t)((CRC3 ? 8U : 0U) + (CRC2 ? 4U : 0U) +
                      (CRC1 ? 2U : 0U) + (CRC0 ? 1U : 0U));
}
