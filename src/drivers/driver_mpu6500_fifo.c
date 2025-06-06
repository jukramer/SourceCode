/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_mpu6500_fifo.c
 * @brief     driver mpu6500 fifo source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2024-07-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2024/07/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_mpu6500_fifo.h"

static mpu6500_handle_t gs_handle; /**< mpu6500 handle */

/**
 * @brief  fifo irq
 * @return status code
 *         - 0 success
 *         - 1 run failed
 * @note   none
 */
uint8_t mpu6500_fifo_irq_handler(void)
{
    if (mpu6500_irq_handler(&gs_handle) != 0)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief     fifo example init
 * @param[in] interface used interface
 * @param[in] addr_pin iic device address
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      spi can't read magnetometer data
 */
uint8_t mpu6500_fifo_init(mpu6500_interface_t interface, mpu6500_address_t addr_pin)
{
    uint8_t res;

    /* link interface function */
    DRIVER_MPU6500_LINK_INIT(&gs_handle, mpu6500_handle_t);
    DRIVER_MPU6500_LINK_IIC_INIT(&gs_handle, mpu6500_interface_iic_init);
    DRIVER_MPU6500_LINK_IIC_DEINIT(&gs_handle, mpu6500_interface_iic_deinit);
    DRIVER_MPU6500_LINK_IIC_READ(&gs_handle, mpu6500_interface_iic_read);
    DRIVER_MPU6500_LINK_IIC_WRITE(&gs_handle, mpu6500_interface_iic_write);
    DRIVER_MPU6500_LINK_SPI_INIT(&gs_handle, mpu6500_interface_spi_init);
    DRIVER_MPU6500_LINK_SPI_DEINIT(&gs_handle, mpu6500_interface_spi_deinit);
    DRIVER_MPU6500_LINK_SPI_READ(&gs_handle, mpu6500_interface_spi_read);
    DRIVER_MPU6500_LINK_SPI_WRITE(&gs_handle, mpu6500_interface_spi_write);
    DRIVER_MPU6500_LINK_DELAY_MS(&gs_handle, mpu6500_interface_delay_ms);
    DRIVER_MPU6500_LINK_DEBUG_PRINT(&gs_handle, mpu6500_interface_debug_print);
    DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&gs_handle, mpu6500_interface_receive_callback);

    /* set the interface */
    res = mpu6500_set_interface(&gs_handle, interface);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interface failed.\n");

        return 1;
    }

    /* set the addr pin */
    res = mpu6500_set_addr_pin(&gs_handle, addr_pin);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set addr pin failed.\n");

        return 1;
    }

    /* init */
    res = mpu6500_init(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: init failed.\n");

        return 1;
    }

    /* delay 100 ms */
    mpu6500_interface_delay_ms(100);

    /* disable sleep */
    res = mpu6500_set_sleep(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sleep failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* if spi interface, disable iic interface */
    if (interface == MPU6500_INTERFACE_SPI)
    {
        /* disable iic */
        res = mpu6500_set_disable_iic_slave(&gs_handle, MPU6500_BOOL_TRUE);
        if (res != 0)
        {
            mpu6500_interface_debug_print("mpu6500: set disable iic slave failed.\n");
            (void)mpu6500_deinit(&gs_handle);

            return 1;
        }
    }

    /* set fifo 1024kb */
    res = mpu6500_set_fifo_1024kb(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo 1024kb failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default clock source */
    res = mpu6500_set_clock_source(&gs_handle, MPU6500_FIFO_DEFAULT_CLOCK_SOURCE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set clock source failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default rate */
    res = mpu6500_set_sample_rate_divider(&gs_handle, MPU6500_FIFO_SAMPLE_RATE_DIVIDER);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set sample rate divider failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default accelerometer range */
    res = mpu6500_set_accelerometer_range(&gs_handle, MPU6500_FIFO_DEFAULT_ACCELEROMETER_RANGE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer range failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default gyroscope range */
    res = mpu6500_set_gyroscope_range(&gs_handle, MPU6500_FIFO_DEFAULT_GYROSCOPE_RANGE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope range failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable temperature sensor */
    res = mpu6500_set_ptat(&gs_handle, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set ptat failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default wake up */
    res = mpu6500_set_cycle_wake_up(&gs_handle, MPU6500_FIFO_DEFAULT_CYCLE_WAKE_UP);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set cycle wake up failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable acc x */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable acc y */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable acc z */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_ACC_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable gyro x */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable gyro y */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable gyro z */
    res = mpu6500_set_standby_mode(&gs_handle, MPU6500_SOURCE_GYRO_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set standby mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable gyroscope x test */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable gyroscope y test */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable gyroscope z test */
    res = mpu6500_set_gyroscope_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope test failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable accelerometer x test */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_X, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable accelerometer y test */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Y, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable accelerometer z test */
    res = mpu6500_set_accelerometer_test(&gs_handle, MPU6500_AXIS_Z, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer test failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* disable temp fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_TEMP, MPU6500_BOOL_FALSE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable xg fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_XG, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable yg fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_YG, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable zg fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ZG, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable accel fifo */
    res = mpu6500_set_fifo_enable(&gs_handle, MPU6500_FIFO_ACCEL, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo enable failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default interrupt level */
    res = mpu6500_set_interrupt_level(&gs_handle, MPU6500_FIFO_DEFAULT_INTERRUPT_PIN_LEVEL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default interrupt pin type */
    res = mpu6500_set_interrupt_pin_type(&gs_handle, MPU6500_FIFO_DEFAULT_INTERRUPT_PIN_TYPE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt pin type failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default motion */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_MOTION, MPU6500_FIFO_DEFAULT_INTERRUPT_MOTION);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default fifo overflow */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FIFO_OVERFLOW, MPU6500_FIFO_DEFAULT_INTERRUPT_FIFO_OVERFLOW);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default dmp interrupt */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DMP, MPU6500_FIFO_DEFAULT_INTERRUPT_DMP);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default fsync int */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_FSYNC_INT, MPU6500_FIFO_DEFAULT_INTERRUPT_FSYNC_INT);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default data ready */
    res = mpu6500_set_interrupt(&gs_handle, MPU6500_INTERRUPT_DATA_READY, MPU6500_FIFO_DEFAULT_INTERRUPT_DATA_READY);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default latch */
    res = mpu6500_set_interrupt_latch(&gs_handle, MPU6500_FIFO_DEFAULT_INTERRUPT_LATCH);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt latch failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default interrupt read clear */
    res = mpu6500_set_interrupt_read_clear(&gs_handle, MPU6500_FIFO_DEFAULT_INTERRUPT_READ_CLEAR);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set interrupt read clear failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default extern sync */
    res = mpu6500_set_extern_sync(&gs_handle, MPU6500_FIFO_DEFAULT_EXTERN_SYNC);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set extern sync failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default fsync interrupt */
    res = mpu6500_set_fsync_interrupt(&gs_handle, MPU6500_FIFO_DEFAULT_FSYNC_INTERRUPT);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default fsync interrupt level */
    res = mpu6500_set_fsync_interrupt_level(&gs_handle, MPU6500_FIFO_DEFAULT_FSYNC_INTERRUPT_LEVEL);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fsync interrupt level failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default iic master */
    res = mpu6500_set_iic_master(&gs_handle, MPU6500_FIFO_DEFAULT_IIC_MASTER);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic master failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default iic bypass */
    res = mpu6500_set_iic_bypass(&gs_handle, MPU6500_FIFO_DEFAULT_IIC_BYPASS);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set iic bypass failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default gyro standby */
    res = mpu6500_set_gyro_standby(&gs_handle, MPU6500_FIFO_DEFAULT_GYROSCOPE_STANDBY);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyro standby failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default fifo mode */
    res = mpu6500_set_fifo_mode(&gs_handle, MPU6500_FIFO_DEFAULT_FIFO_MODE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo mode failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default gyroscope choice */
    res = mpu6500_set_gyroscope_choice(&gs_handle, MPU6500_FIFO_DEFAULT_GYROSCOPE_CHOICE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set gyroscope choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default low pass filter */
    res = mpu6500_set_low_pass_filter(&gs_handle, MPU6500_FIFO_DEFAULT_LOW_PASS_FILTER);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default accelerometer choice */
    res = mpu6500_set_accelerometer_choice(&gs_handle, MPU6500_FIFO_DEFAULT_ACCELEROMETER_CHOICE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer choice failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default accelerometer low pass filter */
    res = mpu6500_set_accelerometer_low_pass_filter(&gs_handle, MPU6500_FIFO_DEFAULT_ACCELEROMETER_LOW_PASS_FILTER);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accelerometer low pass filter failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default low power accel output rate */
    res = mpu6500_set_low_power_accel_output_rate(&gs_handle, MPU6500_FIFO_DEFAULT_LOW_POWER_ACCEL_OUTPUT_RATE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set low power accel output rate failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default wake on motion */
    res = mpu6500_set_wake_on_motion(&gs_handle, MPU6500_FIFO_DEFAULT_WAKE_ON_MOTION);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set wake on motion failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* set the default accel compare with previous sample */
    res = mpu6500_set_accel_compare_with_previous_sample(&gs_handle, MPU6500_FIFO_DEFAULT_ACCELEROMETER_COMPARE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set accel compare with previous sample failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* enable fifo */
    res = mpu6500_set_fifo(&gs_handle, MPU6500_BOOL_TRUE);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: set fifo failed.\n");
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    /* force fifo reset */
    res = mpu6500_force_fifo_reset(&gs_handle);
    if (res != 0)
    {
        mpu6500_interface_debug_print("mpu6500: force fifo reset failed.\n");
        if (interface == MPU6500_INTERFACE_IIC)
        {
            (void)mpu6500_set_fifo(&gs_handle, MPU6500_BOOL_FALSE);
        }
        (void)mpu6500_deinit(&gs_handle);

        return 1;
    }

    return 0;
}

/**
 * @brief         fifo example read
 * @param[out]    **accel_raw pointer to an accel raw data buffer
 * @param[out]    **accel_g pointer to a converted accel data buffer
 * @param[out]    **gyro_raw pointer to an gyro raw data buffer
 * @param[out]    **gyro_dps pointer to a converted gyro data buffer
 * @param[in,out] *len pointer to a length buffer
 * @return        status code
 *                - 0 success
 *                - 1 read failed
 * @note          none
 */
uint8_t mpu6500_fifo_read(int16_t (*accel_raw)[3], float (*accel_g)[3],
                          int16_t (*gyro_raw)[3], float (*gyro_dps)[3],
                          uint16_t *len)
{
    /* fifo read */
    if (mpu6500_read(&gs_handle, accel_raw, accel_g,
                     gyro_raw, gyro_dps, len) != 0)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief  fifo example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t mpu6500_fifo_deinit(void)
{
    /* deinit */
    if (mpu6500_deinit(&gs_handle) != 0)
    {
        return 1;
    }

    return 0;
}