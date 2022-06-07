/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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
* Defines peripherals available for use on CYBT-343026-EVAL board rev 2.0.
*
*
*/

#pragma once

/** \addtogroup Platfrom config - Peripherals pin configuration
*   \ingroup HardwareDrivers
*/
/*! @{ */

/******************************************************
 *                   Enumerations
 ******************************************************/

#include "wiced_bt_types.h"
#include "wiced_hal_gpio.h"

extern void platform_led_init( void );

typedef struct
{
    wiced_bt_gpio_numbers_t led_gpio;
    uint32_t led_config;
    uint32_t led_default_state;
}wiced_led_config_t;

typedef enum
{
    WICED_PLATFORM_LED_1,
    WICED_PLATFORM_LED_MAX
}wiced_platform_led_t;

#define HCI_UART_DEFAULT_BAUD   115200   /* default baud rate is 115200, that is max supported by dual-channel Cypress USB bridge */
#define HCI_UART_MAX_BAUD       1000000

#define WICED_GPIO_BUTTON                                   WICED_P00      /* pin for button interrupts */
#define WICED_GPIO_PIN_BUTTON                               WICED_GPIO_BUTTON
#define WICED_GPIO_BUTTON_DEFAULT_STATE              GPIO_PIN_OUTPUT_LOW

/* x can be GPIO_EN_INT_RISING_EDGE or GPIO_EN_INT_FALLING_EDGE or GPIO_EN_INT_BOTH_EDGE */
#define WICED_GPIO_BUTTON_SETTINGS(x)                       ( GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | x )
#define WICED_BUTTON_PRESSED_VALUE                 1

#define WICED_GPIO_PIN_LED1                       WICED_P28      /* pin for LED (P2, P28, P37)                  */
#define WICED_GPIO_LED_SETTINGS                   (GPIO_OUTPUT_ENABLE | GPIO_PULL_UP)
#define WICED_GPIO_LED_ON_VAL                     0

#define WICED_PUART_TXD                           WICED_P31             /* ARM GPIO pin for PUART TXD         */
#define WICED_PUART_RXD                           WICED_P04             /* ARM GPIO pin for PUART RXD         */
#define WICED_PUART_CTS                           WICED_P03             /* pin for PUART CTS (I2C_SCL, P3, P29, P35)   */
#define WICED_PUART_RTS                           WICED_P30             /* pin for PUART RTS (BT_GPIO_7, P30)          */

/* @} */

/** \addtogroup Platfrom config - Default flash(i.e. flash exists on WICED eval boards) configuration.
*   \ingroup HardwareDrivers
*/
/*! @{ */
/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 *  Recommend to use 4K sector flash.
 */
#if defined(USE_256K_SECTOR_SIZE)
  #define FLASH_SECTOR_SIZE         (256*1024)
  #define FLASH_SIZE                (256*FLASH_SECTOR_SIZE)
#else
  #define FLASH_SECTOR_SIZE         (4*1024)
  #define FLASH_SIZE                0x80000 // 512  kbyte/4M Bit Sflash for new tag boards
#endif

/** Number of sectors reserved from the end of the flash for the application
 *  specific purpose(for ex: to log the crash dump). By default no reservation
 Note:- 16K of flash is used for internal firmware operation.
 (Remaining of the flash - reservation) can be divided equally and used for active
 and upgradable firmware. So, application should care the OTA firmware(application+patch)
 size while reserving using below.
 */
#define APPLICATION_SPECIFIC_FLASH_RESERVATION  0

/**
 * platform_puart_flow_control_pin_init
 *
 * Unbond pads to prepare RTS/CTS flow control
 */
void platform_puart_flow_control_pin_init(void);

/**
 * Platform-specific functions to disable pads bonded to pins
 *
*/
void unbond_P11(void);
void unbond_P30(void);
void unbond_P35(void);

/* @} */
