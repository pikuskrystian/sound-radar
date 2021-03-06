/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v9.0
processor: LPC55S69
package_id: LPC55S69JBD100
mcu_data: ksdk2_0
processor_version: 9.0.2
pin_labels:
- {pin_num: '81', pin_signal: PIO0_2/FC3_TXD_SCL_MISO_WS/CT_INP1/SCT0_OUT0/SCT_GPI2/SECURE_GPIO0_2, label: LCD_DC, identifier: LCD_DC}
- {pin_num: '73', pin_signal: PIO1_28/FC7_SCK/SD0_D5/CT_INP2/PLU_IN3, label: LCD_RST, identifier: LCD_RST}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33_core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '92', peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI_DATA/SD1_D2/CTIMER2_MAT3/SCT0_OUT8/CMP0_OUT/PLU_OUT2/SECURE_GPIO0_29,
    mode: inactive, slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '94', peripheral: FLEXCOMM0, signal: TXD_SCL_MISO_WS, pin_signal: PIO0_30/FC0_TXD_SCL_MISO_WS/SD1_D3/CTIMER0_MAT0/SCT0_OUT9/SECURE_GPIO0_30, mode: inactive,
    slew_rate: standard, invert: disabled, open_drain: disabled}
  - {pin_num: '20', peripheral: ADC0, signal: 'CH, 0', pin_signal: PIO0_23/MCLK/CTIMER1_MAT2/CTIMER3_MAT3/SCT0_OUT4/FC0_CTS_SDA_SSEL0/SD1_D1/SECURE_GPIO0_23/ADC0_0}
  - {pin_num: '14', peripheral: ADC0, signal: 'CH, 8', pin_signal: PIO0_16/FC4_TXD_SCL_MISO_WS/CLKOUT/CT_INP4/SECURE_GPIO0_16/ADC0_8}
  - {pin_num: '86', peripheral: FLEXCOMM3, signal: CTS_SDA_SSEL0, pin_signal: PIO0_4/FC4_SCK/CT_INP12/SCT_GPI4/FC3_CTS_SDA_SSEL0/SECURE_GPIO0_4}
  - {pin_num: '83', peripheral: FLEXCOMM3, signal: RXD_SDA_MOSI_DATA, pin_signal: PIO0_3/FC3_RXD_SDA_MOSI_DATA/CTIMER0_MAT1/SCT0_OUT1/SCT_GPI3/SECURE_GPIO0_3}
  - {pin_num: '89', peripheral: FLEXCOMM3, signal: SCK, pin_signal: PIO0_6/FC3_SCK/CT_INP13/CTIMER4_MAT0/SCT_GPI6/SECURE_GPIO0_6}
  - {pin_num: '81', peripheral: GPIO, signal: 'PIO0, 2', pin_signal: PIO0_2/FC3_TXD_SCL_MISO_WS/CT_INP1/SCT0_OUT0/SCT_GPI2/SECURE_GPIO0_2, direction: OUTPUT, mode: inactive}
  - {pin_num: '73', peripheral: GPIO, signal: 'PIO1, 28', pin_signal: PIO1_28/FC7_SCK/SD0_D5/CT_INP2/PLU_IN3, direction: OUTPUT, gpio_init_state: 'true'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void BOARD_InitPins(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);

    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t LCD_DC_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_2 (pin 81)  */
    GPIO_PinInit(BOARD_INITPINS_LCD_DC_GPIO, BOARD_INITPINS_LCD_DC_PORT, BOARD_INITPINS_LCD_DC_PIN, &LCD_DC_config);

    gpio_pin_config_t gpio0_pin14_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_16 (pin 14)  */
    GPIO_PinInit(GPIO, 0U, 16U, &gpio0_pin14_config);

    gpio_pin_config_t gpio0_pin20_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_23 (pin 20)  */
    GPIO_PinInit(GPIO, 0U, 23U, &gpio0_pin20_config);

    gpio_pin_config_t LCD_RST_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO1_28 (pin 73)  */
    GPIO_PinInit(BOARD_INITPINS_LCD_RST_GPIO, BOARD_INITPINS_LCD_RST_PORT, BOARD_INITPINS_LCD_RST_PIN, &LCD_RST_config);

    IOCON->PIO[0][16] = ((IOCON->PIO[0][16] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_ASW_MASK)))

                         /* Selects pin function.
                          * : PORT016 (pin 14) is configured as ADC0_8. */
                         | IOCON_PIO_FUNC(PIO0_16_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Inactive.
                          * Inactive (no pull-down/pull-up resistor enabled). */
                         | IOCON_PIO_MODE(PIO0_16_MODE_INACTIVE)

                         /* Select Digital mode.
                          * : Disable digital mode.
                          * Digital input set to 0. */
                         | IOCON_PIO_DIGIMODE(PIO0_16_DIGIMODE_ANALOG)

                         /* Analog switch input control.
                          * : For all pins except PIO0_9, PIO0_11, PIO0_12, PIO0_15, PIO0_18, PIO0_31, PIO1_0 and
                          * PIO1_9 analog switch is closed (enabled). */
                         | IOCON_PIO_ASW(PIO0_16_ASW_VALUE1));

    IOCON->PIO[0][2] = ((IOCON->PIO[0][2] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT02 (pin 81) is configured as PIO0_2. */
                        | IOCON_PIO_FUNC(PIO0_2_FUNC_ALT0)

                        /* Selects function mode (on-chip pull-up/pull-down resistor control).
                         * : Inactive.
                         * Inactive (no pull-down/pull-up resistor enabled). */
                        | IOCON_PIO_MODE(PIO0_2_MODE_INACTIVE)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_2_DIGIMODE_DIGITAL));

    IOCON->PIO[0][23] = ((IOCON->PIO[0][23] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_ASW_MASK)))

                         /* Selects pin function.
                          * : PORT023 (pin 20) is configured as ADC0_0. */
                         | IOCON_PIO_FUNC(PIO0_23_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Inactive.
                          * Inactive (no pull-down/pull-up resistor enabled). */
                         | IOCON_PIO_MODE(PIO0_23_MODE_INACTIVE)

                         /* Select Digital mode.
                          * : Disable digital mode.
                          * Digital input set to 0. */
                         | IOCON_PIO_DIGIMODE(PIO0_23_DIGIMODE_ANALOG)

                         /* Analog switch input control.
                          * : For all pins except PIO0_9, PIO0_11, PIO0_12, PIO0_15, PIO0_18, PIO0_31, PIO1_0 and
                          * PIO1_9 analog switch is closed (enabled). */
                         | IOCON_PIO_ASW(PIO0_23_ASW_VALUE1));

    const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    IOCON->PIO[0][3] = ((IOCON->PIO[0][3] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT03 (pin 83) is configured as FC3_RXD_SDA_MOSI_DATA. */
                        | IOCON_PIO_FUNC(PIO0_3_FUNC_ALT1)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_3_DIGIMODE_DIGITAL));

    const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
                                         IOCON_PIO_FUNC1 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
    IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);

    IOCON->PIO[0][4] = ((IOCON->PIO[0][4] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT04 (pin 86) is configured as FC3_CTS_SDA_SSEL0. */
                        | IOCON_PIO_FUNC(0x08u)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_4_DIGIMODE_DIGITAL));

    IOCON->PIO[0][6] = ((IOCON->PIO[0][6] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT06 (pin 89) is configured as FC3_SCK. */
                        | IOCON_PIO_FUNC(PIO0_6_FUNC_ALT1)

                        /* Select Digital mode.
                         * : Enable Digital mode.
                         * Digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO0_6_DIGIMODE_DIGITAL));

    IOCON->PIO[1][28] = ((IOCON->PIO[1][28] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT128 (pin 73) is configured as PIO1_28. */
                         | IOCON_PIO_FUNC(PIO1_28_FUNC_ALT0)

                         /* Select Digital mode.
                          * : Enable Digital mode.
                          * Digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_28_DIGIMODE_DIGITAL));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
