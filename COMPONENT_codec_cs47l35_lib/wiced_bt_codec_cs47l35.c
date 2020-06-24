/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file implements an SPI driver to the CS47L35.
 * It provides initialization functions for the codec.
 *
 */

#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "wiced_bt_sdp.h"
#include "wiced_hal_nvram.h"
#if defined(CYW20706A2) || defined(CYW43012C0)
#include "wiced_hal_platform.h"
#else
#include "wiced_platform.h"
#endif
#ifndef CYW43012C0
#include "wiced_hal_mia.h"
#endif

#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_audio.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_rtos.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_pcm.h"
#include "wiced_hal_gpio.h"

#include "wiced_bt_codec_cs47l35.h"

#define BHAM_SPI_FREQUENCY                          (1000000) /* 24 MHz */

//audio shield2 vals
#define BHAM_SPI_MASTER_P38_CLK_P28_MOSI_P29_MISO   (0x00111d10) /* Macro for SPI master pin configurations */

#if AUDIO_SHIELD_EVK_VER==2

#define BHAM_I2S_DO                             WICED_P06
#define BHAM_I2S_DI                             WICED_P16
#define BHAM_I2S_WS                             WICED_P15
#define BHAM_I2S_CLK                            WICED_P07

#define BHAM_MCLK1_PIN                          WICED_P32
#define BHAM_CODEC_CS_PIN                       WICED_P34
#define BHAM_NIRQ_PIN                           WICED_P33
#define BHAM_PA_CTX_PIN                         WICED_P35
#define BHAM_PA_CRX_PIN                         WICED_P36

#elif AUDIO_SHIELD_EVK_VER==3

#define BHAM_I2S_DO                             WICED_P28
#define BHAM_I2S_DI                             WICED_P01
#define BHAM_I2S_WS                             WICED_P07
#define BHAM_I2S_CLK                            WICED_P38

#define BHAM_MCLK1_PIN                          WICED_P33
#define BHAM_CODEC_CS_PIN                       WICED_P25
#define BHAM_NIRQ_PIN                           WICED_P34
#define BHAM_PA_CTX_PIN                         WICED_P06
#define BHAM_PA_CRX_PIN                         WICED_P09

#else
#error unexpected AUDIO_SHIELD_EVK_VER
#endif

#define BHAM_I2C_SDA_PIN                        WICED_P01
#define BHAM_I2C_SCL_PIN                        WICED_P00

#define BYTE0(N)                        ((uint8_t)((N) >>  0))
#define BYTE1(N)                        ((uint8_t)((N) >>  8))
#define BYTE2(N)                        ((uint8_t)((N) >> 16))
#define BYTE3(N)                        ((uint8_t)((N) >> 24))

/* Codec register address width is 4 bytes for SPI access */
#define CODEC_SPI_ADD_SIZE              (4)
/* Codec 16-bit padding phase for SPI access */
#define CODEC_SPI_PADDING_SIZE          (2)
/* Codec register data width for SPI access */
#define CODEC_SPI_DATA16_SIZE           (2)
#define CODEC_SPI_DATA32_SIZE           (4)

/* Chunks size in bytes, used with driver_codec_buffer_write() and driver_codec_buffer_read() */
#define CODEC_TEMP_BUFF_SIZE            (1024)

#define CODEC_MARLEY_ID                     ((uint16_t)0x6360)

#define CODEC_GPIO5_CTRL_1_ADD              ((uint16_t)0x1708)
#define CODEC_GPIO5_CTRL_2_ADD              ((uint16_t)0x1709)

#define CODEC_IRQ1_STATUS_1_ADD             ((uint16_t)0x1800)
#define CODEC_BOOT_DONE_EINT1_ADD           CODEC_IRQ1_STATUS_1_ADD
#define CODEC_BOOT_DONE_EINT1_MASK          ((uint16_t)0x0080)

#define CODEC_IRQ2_STATUS_9_ADD             ((uint16_t)0x1908)
#define CODEC_DRC2_SIG_DET_EINT2_ADD        CODEC_IRQ2_STATUS_9_ADD
#define CODEC_DRC2_SIG_DET_EINT2_MASK       ((uint16_t)0x0002)

#define CODEC_IRQ1_STATUS_11_ADD            ((uint16_t)0x180A)
#define CODEC_DSP_IRQ1_EINT1_ADD            CODEC_IRQ1_STATUS_11_ADD
#define CODEC_DSP_IRQ1_EINT1_MASK           ((uint16_t)0x0001)

#define CODEC_IRQ1_STATUS_17_ADD            ((uint16_t)0x1810)
#define CODEC_IRQ1_MASK_17_ADD              ((uint16_t)0x1850)
#define CODEC_IM_GPIO_EINT1_ADD             CODEC_IRQ1_STATUS_17_ADD
#define CODEC_IM_GPIO2_EINT1_MASK           ((uint16_t)0x0002)
#define CODEC_IM_GPIO3_EINT1_MASK           ((uint16_t)0x0004)
#define CODEC_IM_GPIO4_EINT1_MASK           ((uint16_t)0x0008)

/* CS47L35 register address */
#define CODEC_DAC_DIGITAL_VOLUME_1L         0x0411
#define CODEC_DAC_DIGITAL_VOLUME_1R         0x0415
#define CODEC_DAC_DIGITAL_VOLUME_4L         0x0429
#define CODEC_OUTPUT_ENABLE_1               0x0400
#define CODEC_OUT1LMIX_INPUT_1_SOURCE       0x0680
#define CODEC_OUT1RMIX_INPUT_1_SOURCE       0x0688
#define CODEC_OUT4LMIX_INPUT_1_SOURCE       0x06B0
#define CODEC_SYSTEM_CLOCK_1                0x0101
#define CODEC_SAMPLE_RATE_1                 0x0102
#define CODEC_IN1L_CONTROL                  0x0310
#define CODEC_IN1R_CONTROL                  0x0314
#define CODEC_IN2L_CONTROL                  0x0318
#define CODEC_IN2R_CONTROL                  0x031C

#define iocfg_fcn_p0_adr                               0x00338400

static uint8_t p_spi_tx_buffer[CODEC_SPI_DATA32_SIZE];
static wiced_bool_t codec_cs47l35_initialized = WICED_FALSE;
static cs47l35_output_t codec_cs47l35_output = CS47L35_OUTPUT_HEADSET;

void codec_cs47l35_set_sink(cs47l35_output_t output, uint8_t mono_input);
void codec_cs47l35_set_sample_rate(uint32_t sample_rate);
void codec_write_reg_config( codec_reg * reg_cfgs, uint32_t num_cfgs);

void platform_bham_codec_marley_ctrl_bus_init(void)
{
    if (codec_cs47l35_initialized == WICED_FALSE)
    {
        wiced_hal_pspi_init(SPI2, BHAM_SPI_FREQUENCY, SPI_MSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_0);
        REG32(iocfg_fcn_p0_adr + (4 * BHAM_CODEC_CS_PIN)) = 0;
        wiced_hal_gpio_configure_pin(BHAM_CODEC_CS_PIN, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);

        driver_codec_reset();
        wiced_rtos_delay_milliseconds(10, ALLOW_THREAD_TO_SLEEP);
        while (1)
        {
            if (driver_codec_nirq_check() != 0)
            {
                uint16_t id;

                id = driver_codec_id_get();
                if (id == CODEC_MARLEY_ID)
                {
                    WICED_BT_TRACE("Codec CS47L35 detected\n");
                }
                else
                {
                    WICED_BT_TRACE("Codec CS47L35 not detected 0x%x\n", id);
                }
                break;
            }
            else
            {
                WICED_BT_TRACE("Codec CS47L35 not detected\n");
                wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP);
            }
        }
        codec_write_reg_config( power_up_codec_config, power_up_codec_config_len);

        codec_cs47l35_initialized = WICED_TRUE;
    }
}

void platform_bham_codec_marley_write_cmd(uint32_t address, uint16_t tx_length, const uint8_t *p_tx_buffer)
{
    uint8_t p_spi_tx_buffer[6];

    wiced_hal_gpio_set_pin_output(BHAM_CODEC_CS_PIN, GPIO_PIN_OUTPUT_LOW);
    p_spi_tx_buffer[0] = (uint8_t) BYTE3(address);
    p_spi_tx_buffer[1] = (uint8_t) BYTE2(address);
    p_spi_tx_buffer[1] = (uint8_t) BYTE2(address);
    p_spi_tx_buffer[2] = (uint8_t) BYTE1(address);
    p_spi_tx_buffer[3] = (uint8_t) BYTE0(address);
    p_spi_tx_buffer[4] = (uint8_t) 0; /* 16-bit padding phase */
    p_spi_tx_buffer[5] = (uint8_t) 0; /* 16-bit padding phase */
    /* Send address */
    wiced_hal_pspi_tx_data(SPI2, 6, p_spi_tx_buffer);
    /* Write data */
    wiced_hal_pspi_tx_data(SPI2, tx_length, p_tx_buffer);
    wiced_hal_gpio_set_pin_output(BHAM_CODEC_CS_PIN, GPIO_PIN_OUTPUT_HIGH);
}

void platform_bham_codec_marley_read_cmd(uint32_t address, uint16_t rx_length, uint8_t *p_rx_buffer)
{
    uint8_t p_spi_tx_buffer[6];

    wiced_hal_gpio_set_pin_output(BHAM_CODEC_CS_PIN, GPIO_PIN_OUTPUT_LOW);
    /* force read / write bit to read */
    p_spi_tx_buffer[0] = (uint8_t) (BYTE3(address) | 0x80);
    p_spi_tx_buffer[1] = (uint8_t) BYTE2(address);
    p_spi_tx_buffer[1] = (uint8_t) BYTE2(address);
    p_spi_tx_buffer[2] = (uint8_t) BYTE1(address);
    p_spi_tx_buffer[3] = (uint8_t) BYTE0(address);
    p_spi_tx_buffer[4] = (uint8_t) 0; /* 16-bit padding phase */
    p_spi_tx_buffer[5] = (uint8_t) 0; /* 16-bit padding phase */
    /* Send address */
    wiced_hal_pspi_tx_data(SPI2, 6, p_spi_tx_buffer);
    /* Read data */
    wiced_hal_pspi_rx_data(SPI2, rx_length, p_rx_buffer);
    wiced_hal_gpio_set_pin_output(BHAM_CODEC_CS_PIN, GPIO_PIN_OUTPUT_HIGH);
}

void codec_write_reg_config( codec_reg * reg_cfgs, uint32_t num_cfgs)
{
    int i;

    for( i = 0; i < num_cfgs; i++)
    {
        //special cse inserted into reg config to add a delay
        if( (reg_cfgs[i].addr == 0xffff) && (reg_cfgs[i].val == 0) )
        {
            WICED_BT_TRACE("INSERT_DELAY_MS\n");
            wiced_rtos_delay_milliseconds(10, ALLOW_THREAD_TO_SLEEP);
            continue;
        }

        driver_codec_register_write(reg_cfgs[i].addr, reg_cfgs[i].val);
    }
}


void driver_codec_mute_disable_all_output(void)
{
    /* Mute all outputs */
    driver_codec_write16(0x415, 0x168);
    driver_codec_write16(0x429, 0x100);
    driver_codec_write16(0x411, 0x368);
    /* disable all outputs */
    driver_codec_write16(0x400, 0);
}


void driver_codec_reset(void)
{
    WICED_BT_TRACE("DRIVER_CODEC reset\n");

    /* SW reset codec */
    driver_codec_write16(0x0, 0);
}

#define CODEC_MARLEY_MEMORY_MAP_EXTENDED_ADD    (0x3000)
void driver_codec_register_write(uint32_t address, uint32_t value)
{
    if (address >= CODEC_MARLEY_MEMORY_MAP_EXTENDED_ADD)
    {
        driver_codec_write32(address, value);
    }
    else
    {
        driver_codec_write16(address, (uint16_t)value);
    }
}

void driver_codec_write16(uint32_t address, uint16_t value)
{
    //WICED_BT_TRACE("DRIVER_CODEC W16 %04X:%04X\n", (uint32_t)address, (uint32_t)value);
    p_spi_tx_buffer[0] = (uint8_t) BYTE1(value);
    p_spi_tx_buffer[1] = (uint8_t) BYTE0(value);
    /* Send data with SPI */
    platform_bham_codec_marley_write_cmd(address, 2, p_spi_tx_buffer);
}

void driver_codec_write32(uint32_t address, uint32_t value)
{
    //WICED_BT_TRACE("DRIVER_CODEC W32 %X:%X\n", (uint32_t)address, (uint32_t)value);
    p_spi_tx_buffer[0] = (uint8_t) BYTE3(value);
    p_spi_tx_buffer[1] = (uint8_t) BYTE2(value);
    p_spi_tx_buffer[2] = (uint8_t) BYTE1(value);
    p_spi_tx_buffer[3] = (uint8_t) BYTE0(value);
    /* Send data with SPI */
    platform_bham_codec_marley_write_cmd(address, 4, p_spi_tx_buffer);
}

uint32_t driver_codec_register_read(uint32_t address)
{
    uint32_t value;

    if (address >= CODEC_MARLEY_MEMORY_MAP_EXTENDED_ADD)
    {
        value = driver_codec_read32(address);
    }
    else
    {
        value = driver_codec_read16(address);
    }

    return value;
}

uint16_t driver_codec_read16(uint32_t address)
{
    uint8_t p_spi_rx_buffer[CODEC_SPI_DATA32_SIZE];
    uint16_t value;

    /* Send data with SPI */
    platform_bham_codec_marley_read_cmd(address, 2, p_spi_rx_buffer);
    value = (uint16_t)p_spi_rx_buffer[0];
    value <<= 8;
    value |= p_spi_rx_buffer[1];
    WICED_BT_TRACE("DRIVER_CODEC R16 %X:%04X\n", address, value);

    return value;
}

uint32_t driver_codec_read32(uint32_t address)
{
    uint8_t p_spi_rx_buffer[CODEC_SPI_DATA32_SIZE];
    uint32_t value;

    /* Send data with SPI */
    platform_bham_codec_marley_read_cmd(address, 4, p_spi_rx_buffer);
    value = (uint32_t)p_spi_rx_buffer[0];
    value <<=8;
    value |= p_spi_rx_buffer[1];
    value <<=8;
    value |= p_spi_rx_buffer[2];
    value <<=8;
    value |= p_spi_rx_buffer[3];
    WICED_BT_TRACE("DRIVER_CODEC R32 %X:%08X\n", address, (uint32_t)value);

    return value;
}

uint16_t driver_codec_id_get(void)
{
    uint16_t id;

    id = driver_codec_read16(0x0);
    WICED_BT_TRACE("DRIVER_CODEC id: 0x%04X\n", (uint32_t)id);

    return id;
}

uint8_t driver_codec_nirq_check(void)
{
    uint16_t reg;

    //WICED_BT_TRACE("DRIVER_CODEC check nirq\n");
    reg = driver_codec_read16(CODEC_BOOT_DONE_EINT1_ADD);
    if (reg)
    {
        /* reset flags */
        driver_codec_write16(CODEC_BOOT_DONE_EINT1_ADD, reg);
        if (reg & CODEC_BOOT_DONE_EINT1_MASK)
        {
            WICED_BT_TRACE("CODEC BOOT DONE\n");
            return 1;
        }
    }

    return 0;
}

void wiced_bt_codec_cs47l35_init(cs47l35_stream_type_t stream_type, uint32_t sample_rate)
{
    switch (stream_type)
    {
        case CS47L35_STREAM_A2DP:
            codec_write_reg_config( a2dp_start_stream_codec_config, a2dp_start_stream_codec_config_len);
            codec_cs47l35_set_sink(codec_cs47l35_output, 0);
            break;

        case CS47L35_STREAM_SCO:
            codec_write_reg_config( sco_stream_codec_config, sco_stream_codec_config_len);
            codec_cs47l35_set_sink(codec_cs47l35_output, 1);
            break;

        case CS47L35_STREAM_CAPTURE:
            codec_write_reg_config( a2dp_source_stream_codec_config, a2dp_source_stream_codec_config_len);
            codec_cs47l35_set_sink(codec_cs47l35_output, 1);
            break;
    }
    codec_cs47l35_set_sample_rate(sample_rate);
}

/* Set output volume

    Parameters:
        @left_vol           left volume step 0 ... 0xBF
        @right_vol          right_volume step 0 ... 0xBF

        The output volume will be set to mute for volume step == 0

    Return:                 N/A

    Output Path 1 Digital Volume
    –64 dB to +31.5 dB in 0.5-dB steps
    0x00 = –64dB
    0x01 = –63.5dB
    … (0.5-dB steps)
    0x80 = 0 dB
    … (0.5-dB steps)
    0xBF = +31.5 dB
*/
void wiced_bt_codec_cs47l35_set_output_volume(uint8_t left_vol, uint8_t right_vol)
{
    uint16_t reg;

    uint16_t left_mute = (left_vol == 0) ? 1 : 0;
    uint16_t right_mute = (right_vol == 0) ? 1 : 0;

    WICED_BT_TRACE("%s left_vol:%d right_vol:%d\n", __FUNCTION__, left_vol, right_vol);

    if (left_vol > 0xbf || right_vol > 0xbf)
    {
        return;
    }

    reg = (uint16_t) left_vol | (left_mute << 8) | 0x1 << 9;
    driver_codec_write16(CODEC_DAC_DIGITAL_VOLUME_1L, reg);

    reg = (uint16_t) right_vol | (right_mute << 8) | 0x1 << 9;
    driver_codec_write16(CODEC_DAC_DIGITAL_VOLUME_1R, reg);

    reg = (uint16_t) right_vol | (right_mute << 8) | 0x1 << 9;
    driver_codec_write16(CODEC_DAC_DIGITAL_VOLUME_4L, reg);
}

void wiced_bt_codec_cs47l35_set_input_volume(uint8_t left_vol, uint8_t right_vol)
{
    uint16_t reg;

    WICED_BT_TRACE("%s left_vol:%d right_vol:%d\n", __FUNCTION__, left_vol, right_vol);

    if (left_vol > 31)
    {
        left_vol = 31;
    }

    if (right_vol > 31)
    {
        right_vol = 31;
    }

    reg = 0x9000 | ((left_vol + 0x40) << 1);
    driver_codec_write16(CODEC_IN1L_CONTROL, reg);

    reg = 0x8000 | ((right_vol + 0x40) << 1);
    driver_codec_write16(CODEC_IN1R_CONTROL, reg);

    reg = 0x8000 | ((left_vol + 0x40) << 1);
    driver_codec_write16(CODEC_IN2L_CONTROL, reg);

    reg = 0x8000 | ((right_vol + 0x40) << 1);
    driver_codec_write16(CODEC_IN2R_CONTROL, reg);
}

void wiced_bt_codec_cs47l35_set_sink(cs47l35_output_t output)
{
    codec_cs47l35_output = output;
}

void codec_cs47l35_set_sink(cs47l35_output_t output, uint8_t mono_input)
{
    switch (output)
    {
        case CS47L35_OUTPUT_HEADSET:
            driver_codec_write16(CODEC_OUTPUT_ENABLE_1, 0x0003);
            driver_codec_write16(CODEC_OUT1LMIX_INPUT_1_SOURCE, 0x0028);
            if (mono_input)
            {
                driver_codec_write16(CODEC_OUT1RMIX_INPUT_1_SOURCE, 0x0028);
            }
            else
            {
                driver_codec_write16(CODEC_OUT1RMIX_INPUT_1_SOURCE, 0x0029);
            }
            break;

        case CS47L35_OUTPUT_SPEAKER:
            driver_codec_write16(CODEC_OUTPUT_ENABLE_1, 0x0080);
            driver_codec_write16(CODEC_OUT4LMIX_INPUT_1_SOURCE, 0x0028);
            break;
    }
}

void codec_cs47l35_set_sample_rate(uint32_t sample_rate)
{
    uint16_t reg;

    switch (sample_rate)
    {
    case 12000:
        reg = 0x01;
        break;
    case 24000:
        reg = 0x02;
        break;
    case 48000:
        reg = 0x03;
        break;
    case 96000:
        reg = 0x04;
        break;
    case 192000:
        reg = 0x05;
        break;
    case 11025:
        reg = 0x09;
        break;
    case 22050:
        reg = 0x0a;
        break;
    case 44100:
        reg = 0x0b;
        break;
    case 88200:
        reg = 0x0c;
        break;
    case 176400:
        reg = 0x0d;
        break;
    case 8000:
        reg = 0x11;
        break;
    case 16000:
        reg = 0x12;
        break;
    case 32000:
        reg = 0x13;
        break;
    default:
        /* not support */
        return;
    }

    driver_codec_register_write(CODEC_SYSTEM_CLOCK_1, 0);
    driver_codec_register_write(CODEC_SAMPLE_RATE_1, reg);
    driver_codec_register_write(CODEC_SYSTEM_CLOCK_1, 0x0444);
}
