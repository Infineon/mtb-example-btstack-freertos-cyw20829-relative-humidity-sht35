/*******************************************************************************
 Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 *        Header Files
 *******************************************************************************/
#include "cyhal.h"
#include "sensor_task.h"
#include "cy_retarget_io.h"
#include "timers.h"

#include "app_bt_gatt_handler.h"

#include "mtb_sht3x.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define POLL_TIMER_IN_MSEC              (1000u)
#define POLL_TIMER_FREQ                 (10000)

/* Check if notification is enabled for a valid connection ID */
#define IS_NOTIFIABLE(conn_id, cccd)    (((conn_id)!= 0)? (cccd) & GATT_CLIENT_CONFIG_NOTIFICATION: 0)

#define I2C_CLK_FREQ_HZ                 (400000U)

#define TEMPERATURE_START_INDEX         (0)
#define RELATIVE_HUMIDITY_START_INDEX   (4)

/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
/* HAL structure for I2C */
static cyhal_i2c_t i2c_obj;

/* Handle to timer object */
TimerHandle_t timer_handle;

/******************************************************************************
 *                          Function Prototypes
 ******************************************************************************/
void timer_callback(TimerHandle_t xTimer);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
int32_t sensor_task_init()
{
    int32_t result;

    /* Create timer */
    timer_handle = xTimerCreate("timer",
                                pdMS_TO_TICKS(POLL_TIMER_IN_MSEC),
                                pdTRUE,
                                NULL,
                                timer_callback);
    if (NULL == timer_handle)
    {
        return -1;
    }

    /* Start Timer */
    xTimerStart(timer_handle, 0);

    /* Initialize sensor driver here */

    /* I2C Configuration Structure */
    cyhal_i2c_cfg_t i2c_config = { false, 0, I2C_CLK_FREQ_HZ };

    /* Initialize I2C for SHT35 */
    result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error initializing i2c\n");
        return -2;
    }

    /* Configure the I2C Interface with the desired clock frequency */
    result = cyhal_i2c_configure(&i2c_obj, &i2c_config);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error configuring i2c\n");
        return -3;
    }

    /* Initialize and Configure SHT35 sensor */
    result = mtb_sht3x_init(&i2c_obj, MTB_SHT35_ADDRESS_DEFAULT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error initializing SHT35 sensor\n");
        return -4;
    }

    /* Starts the periodic measurement mode of SHT35 sensor */
    result = mtb_sht3x_start_periodic_measurement(&i2c_obj,
                                                  REPEAT_MEDIUM,
                                                  MPS_ONE_PER_SEC);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error starting periodic measurement\n");
        return -5;
    }

    return 0;
}

void sensor_task(void *pvParameters)
{
    struct sensor_data_t sht3x_data;

    float temperature = 0;
    float humidity    = 0;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Read humidity & temperature data */
        sht3x_data = mtb_sht3x_read(&i2c_obj);

        printf("Temperature: %.2f degC\n", sht3x_data.temperature);
        printf("Relative Humidity: %.2f %%\n", sht3x_data.humidity);

        /* Cast result to float */
        temperature = (float) sht3x_data.temperature;
        humidity    = (float) sht3x_data.humidity;

        /* Copy to ble buffer so reading will return correct data */
        memcpy(&app_xensiv_sensor_shield_sht35[TEMPERATURE_START_INDEX], &temperature, sizeof(float));
        memcpy(&app_xensiv_sensor_shield_sht35[RELATIVE_HUMIDITY_START_INDEX], &humidity, sizeof(float));

        if (IS_NOTIFIABLE (app_bt_conn_id, app_xensiv_sensor_shield_sht35_client_char_config[0]) == 0)
        {
            if(!app_bt_conn_id)
            {
                printf("This device is not connected to a central device\n");
            }else{
                printf("This device is connected to a central device but\n"
                        "GATT client notifications are not enabled\n");
            }
        }
        else
        {
            wiced_bt_gatt_status_t gatt_status;

            /*
            * Sending notification, set the pv_app_context to NULL, since the
            * data 'app_xensiv_sensor_shield_sht35' is not to be freed
            */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_XENSIV_SENSOR_SHIELD_SHT35_VALUE,
                                                                 app_xensiv_sensor_shield_sht35_len,
                                                                 (uint8_t *) app_xensiv_sensor_shield_sht35,
                                                                 NULL);

            printf("Sent notification status 0x%x\n", gatt_status);
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
