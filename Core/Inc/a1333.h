/**
 * @file a1333.h
 * @brief Allegro A1333 Hall-Effect Angle Sensor SPI Driver (STM32 HAL)
 *
 * SPI Mode 3 (CPOL=1, CPHA=1), 16-bit frames, max 10 MHz SCLK.
 * Reads are pipelined: send read command in frame N, receive data in frame N+1.
 */

#ifndef A1333_H
#define A1333_H

#include "stm32g4xx_hal.h"  // STM32G4xx family
#include <stdint.h>
#include <stdbool.h>

/* ── Primary Serial Register Addresses ─────────────────────────────── */
#define A1333_REG_NOP       0x00
#define A1333_REG_EWA       0x02  // Extended Write Address
#define A1333_REG_EWDH      0x04  // Extended Write Data High
#define A1333_REG_EWDL      0x06  // Extended Write Data Low
#define A1333_REG_EWCS      0x08  // Extended Write Control/Status
#define A1333_REG_ERA       0x0A  // Extended Read Address
#define A1333_REG_ERCS      0x0C  // Extended Read Control/Status
#define A1333_REG_ERDH      0x0E  // Extended Read Data High
#define A1333_REG_ERDL      0x10  // Extended Read Data Low
#define A1333_REG_CTRL      0x1E  // Device Control
#define A1333_REG_ANG       0x20  // Current Angle (12-bit)
#define A1333_REG_STA       0x22  // Device Status
#define A1333_REG_ERR       0x24  // Error Flags
#define A1333_REG_WARN      0x26  // Warning Flags
#define A1333_REG_TSEN      0x28  // Temperature Sensor
#define A1333_REG_FIELD     0x2A  // Field Strength (Gauss)
#define A1333_REG_TURNS     0x2C  // Turns Counter
#define A1333_REG_HANG      0x30  // Hysteresis Angle
#define A1333_REG_ANG15     0x32  // 15-bit Angle
#define A1333_REG_ZANG      0x34  // ZCD Angle
#define A1333_REG_IKEY      0x3C  // Key Register (unlock)

/* ── EEPROM Addresses ──────────────────────────────────────────────── */
#define A1333_EEP_PWE       0x18  // PWM Error Enable
#define A1333_EEP_ABI       0x19  // ABI Control
#define A1333_EEP_MSK       0x1A  // Mask Bits
#define A1333_EEP_PWI       0x1B  // PWM Interface Control
#define A1333_EEP_ANG       0x1C  // Angle Config (ORATE, hysteresis, zero offset)
#define A1333_EEP_LPC       0x1D  // Turns counter resolution
#define A1333_EEP_COM       0x1E  // Common config (lock, thresholds, self-test)
#define A1333_EEP_CUS       0x1F  // Customer EEPROM space

/* ── Shadow Memory Offset ──────────────────────────────────────────── */
#define A1333_SHADOW_OFFSET  0x40

/* ── ERR Register Bit Masks ────────────────────────────────────────── */
#define A1333_ERR_WAR       (1 << 11)
#define A1333_ERR_STF       (1 << 10)
#define A1333_ERR_AVG       (1 << 9)
#define A1333_ERR_ABI       (1 << 8)
#define A1333_ERR_PLK       (1 << 7)
#define A1333_ERR_ZIE       (1 << 6)
#define A1333_ERR_EUE       (1 << 5)
#define A1333_ERR_OFE       (1 << 4)
#define A1333_ERR_UVD       (1 << 3)
#define A1333_ERR_UVA       (1 << 2)
#define A1333_ERR_MSL       (1 << 1)
#define A1333_ERR_RST       (1 << 0)

/* ── ANG Register Bit Masks ────────────────────────────────────────── */
#define A1333_ANG_EF_MASK   (1 << 14)
#define A1333_ANG_UV_MASK   (1 << 13)
#define A1333_ANG_P_MASK    (1 << 12)
#define A1333_ANG_ANGLE_MASK 0x0FFF

/* ── STA Register Bit Masks ────────────────────────────────────────── */
#define A1333_STA_AOK       (1 << 0)
#define A1333_STA_BIP       (1 << 1)
#define A1333_STA_BDN       (1 << 4)
#define A1333_STA_SDN       (1 << 5)

/* ── CTRL SPECIAL Values ───────────────────────────────────────────── */
#define A1333_SPECIAL_NONE          0x0
#define A1333_SPECIAL_MARGIN_LO     0x1
#define A1333_SPECIAL_MARGIN_HI     0x2
#define A1333_SPECIAL_TURNS_RESET   0x4
#define A1333_SPECIAL_RELOAD_EEP    0x5
#define A1333_SPECIAL_HARD_RESET    0x7
#define A1333_SPECIAL_CVH_TEST      0x9
#define A1333_SPECIAL_LBIST         0xA
#define A1333_SPECIAL_BOTH_TESTS    0xB

/* ── Return Codes ──────────────────────────────────────────────────── */
typedef enum {
    A1333_OK = 0,
    A1333_ERR_SPI,
    A1333_ERR_TIMEOUT,
    A1333_ERR_PARITY,
    A1333_ERR_DEVICE,
    A1333_ERR_NOT_READY,
    A1333_ERR_UNLOCK_FAIL,
} A1333_Status;

/* ── Device Handle ─────────────────────────────────────────────────── */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} A1333_Handle;

/* ── Core SPI Functions ────────────────────────────────────────────── */
A1333_Status A1333_Init(A1333_Handle *dev, SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin);

A1333_Status A1333_ReadReg(A1333_Handle *dev, uint8_t addr, uint16_t *data);
A1333_Status A1333_WriteReg(A1333_Handle *dev, uint8_t addr, uint8_t data);

/* ── Angle Reading ─────────────────────────────────────────────────── */
A1333_Status A1333_ReadAngle12(A1333_Handle *dev, uint16_t *raw, float *degrees);
A1333_Status A1333_ReadAngle15(A1333_Handle *dev, uint16_t *raw, float *degrees);
A1333_Status A1333_ReadAngleHysteresis(A1333_Handle *dev, uint16_t *raw, float *degrees);

/* ── Diagnostics ───────────────────────────────────────────────────── */
A1333_Status A1333_ReadStatus(A1333_Handle *dev, uint16_t *status);
A1333_Status A1333_ReadErrors(A1333_Handle *dev, uint16_t *errors);
A1333_Status A1333_ReadWarnings(A1333_Handle *dev, uint16_t *warnings);
A1333_Status A1333_ClearErrors(A1333_Handle *dev);
A1333_Status A1333_ClearWarnings(A1333_Handle *dev);
A1333_Status A1333_IsAngleValid(A1333_Handle *dev, bool *valid);

/* ── Temperature & Field ───────────────────────────────────────────── */
A1333_Status A1333_ReadTemperature(A1333_Handle *dev, float *temp_c);
A1333_Status A1333_ReadFieldStrength(A1333_Handle *dev, uint16_t *gauss);

/* ── Turns Counter ─────────────────────────────────────────────────── */
A1333_Status A1333_ReadTurns(A1333_Handle *dev, int16_t *turns);
A1333_Status A1333_ResetTurns(A1333_Handle *dev);

/* ── EEPROM Access ─────────────────────────────────────────────────── */
A1333_Status A1333_Unlock(A1333_Handle *dev);
A1333_Status A1333_ReadEEPROM(A1333_Handle *dev, uint8_t eep_addr, uint32_t *data);
A1333_Status A1333_WriteEEPROM(A1333_Handle *dev, uint8_t eep_addr, uint32_t data);

/* ── Shadow Memory ─────────────────────────────────────────────────── */
A1333_Status A1333_ReadShadow(A1333_Handle *dev, uint8_t eep_addr, uint32_t *data);
A1333_Status A1333_WriteShadow(A1333_Handle *dev, uint8_t eep_addr, uint32_t data);

/* ── Configuration Helpers ─────────────────────────────────────────── */
A1333_Status A1333_SetZeroOffset(A1333_Handle *dev, uint16_t offset_12bit);
A1333_Status A1333_SetRotationDirection(A1333_Handle *dev, bool counter_clockwise);

/* ── CRC (for 20-bit mode) ─────────────────────────────────────────── */
uint8_t A1333_CalculateCRC(uint16_t input);

#endif /* A1333_H */
