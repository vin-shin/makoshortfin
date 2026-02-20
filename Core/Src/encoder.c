#include "encoder.h"
#include "config.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"

static uint16_t s_last_good = 0;
static uint16_t s_debug_rx1 = 0;
static uint16_t s_debug_rx2 = 0;
static uint8_t s_first_read = 1;

void Encoder_Init(void)
{
    LL_GPIO_SetOutputPin(ENC_CS_PORT, ENC_CS_PIN);
}

/* Fast inline SPI3 16-bit transfer with loop-counter timeout */
static inline uint16_t SPI3_Transfer16(uint16_t tx)
{
    uint32_t timeout;

    /* Wait for TXE */
    timeout = SPI_TIMEOUT_LOOPS;
    while (!LL_SPI_IsActiveFlag_TXE(SPI3)) {
        if (--timeout == 0) return 0xFFFF;
    }

    LL_SPI_TransmitData16(SPI3, tx);

    /* Wait for RXNE */
    timeout = SPI_TIMEOUT_LOOPS;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI3)) {
        if (--timeout == 0) return 0xFFFF;
    }

    uint16_t rx = LL_SPI_ReceiveData16(SPI3);

    /* Wait for BSY clear */
    timeout = SPI_TIMEOUT_LOOPS;
    while (LL_SPI_IsActiveFlag_BSY(SPI3)) {
        if (--timeout == 0) return 0xFFFF;
    }

    return rx;
}

/* Read 15-bit angle from A1333 via 2-frame pipelined SPI.
 * Returns raw counts [0, 32767] — no float conversion performed here. */
Encoder_Status_t Encoder_ReadAngle(uint16_t *raw_counts)
{
    /* Frame 1: send ANG15 read command */
    LL_GPIO_ResetOutputPin(ENC_CS_PORT, ENC_CS_PIN);
    uint16_t rx1 = SPI3_Transfer16(A1333_ANG15_CMD);
    LL_GPIO_SetOutputPin(ENC_CS_PORT, ENC_CS_PIN);

    if (rx1 == 0xFFFF) {
        *raw_counts = s_last_good;
        return ENC_ERR_SPI;
    }

    /* CS idle delay >350ns (A1333 spec) */
    volatile uint32_t d = 15;
    while (d--) {}

    /* Frame 2: clock out response with NOP */
    LL_GPIO_ResetOutputPin(ENC_CS_PORT, ENC_CS_PIN);
    uint16_t rx2 = SPI3_Transfer16(A1333_NOP_CMD);
    LL_GPIO_SetOutputPin(ENC_CS_PORT, ENC_CS_PIN);

    if (rx2 == 0xFFFF) {
        *raw_counts = s_last_good;
        return ENC_ERR_SPI;
    }

    uint16_t ang_raw = rx2 & 0x7FFF;
    s_last_good = ang_raw;
    *raw_counts = ang_raw;

    /* Store raw data for debugging (first read only) */
    if (s_first_read) {
        s_first_read = 0;
        s_debug_rx1 = rx1;
        s_debug_rx2 = rx2;
    }

    return ENC_OK;
}

/* Get last SPI debug data */
void Encoder_GetDebugData(uint16_t *rx1, uint16_t *rx2)
{
    *rx1 = s_debug_rx1;
    *rx2 = s_debug_rx2;
}

