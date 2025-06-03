

#include "IMU_EKF/ESKF.h"

using Eigen::Vector3f;

IMU_EKF::ESKF<float> filter;


int main()
{
    //
    // Main loop
    //

    // TOF_XL.startContinuous(50); // Start continuous ranging with 50ms period

    for (int i = 0; i < 4; i++)
    {
        TOF_XS[i].startRangeContinuous(50);
    }

    /**
     *         for (int i = 0; i < 0; i++)
        {
            if (TOF_XS[i].isRangeComplete())
            {
                mm[i] = TOF_XS[i].readRangeResult();
            }

            // float lux = TOF_XS[i].readLux(VL6180X_ALS_GAIN_5);
            // uint8_t range = TOF_XS[i].readRange();
            // uint8_t status = TOF_XS[i].readRangeStatus();
            // printf("Sensor %d - Lux: %.2f, Range: %d mm, Status: %d, ", i, lux, range, status);

            // printf("%d: Range: %d mm | ", i, mm[i]);
        }

                /*if (TOF_XL.readRangeContinuousMillimeters())
        {
            mmXL = TOF_XL.readRangeContinuousMillimeters();
            printf("XL Range: %d mm ", (int)mmXL);
        }*/
    
    uint8_t mm[4] = {};
    float mmXL = 0;

    auto t_prev = time_us_64();
    auto t_end_calib = t_prev + 3000000; // 3s calibration

    int calib_samples = 0;
    bool calibrated = false;

    filter.initWithAcc(0.0f, 0.0f, 0.0f);
    Vector3f accel_bias, gyro_bias;

    while (true)
    {
        auto t1 = time_us_64();

        uint16_t len = 128;
        mpu6500_fifo_read(gs_accel_raw, gs_accel_g, gs_gyro_raw, gs_gyro_dps, &len);
        auto t2 = time_us_64();

        float dt = (t2 - t_prev) / 1e6f;
        t_prev = t2;

        Vector3f accel_raw(gs_accel_g[0][0], gs_accel_g[0][1], gs_accel_g[0][2]);
        Vector3f gyro_raw(gs_gyro_dps[0][0], gs_gyro_dps[0][1], gs_gyro_dps[0][2]);

        if (!calibrated)
        {
            accel_raw[2] -= 1.0f; // Remove gravity from Z axis

            accel_bias += accel_raw;
            gyro_bias += gyro_raw;
            calib_samples++;

            if (t2 > t_end_calib)
            {
                accel_bias *= (1.0f / calib_samples);
                gyro_bias *= (1.0f / calib_samples);
                calibrated = true;
            }

            continue;
        }

        accel_raw -= accel_bias;
        gyro_raw -= gyro_bias;

        filter.predict(dt);
        filter.correctGyr(gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        filter.correctAcc(accel_raw[0], accel_raw[1], accel_raw[2]);

        float roll, pitch, yaw;
        filter.getAttitude(roll, pitch, yaw);

        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f | "
               "Accel: [%.2f, %.2f, %.2f] m/s^2 | "
               "Gyro: [%.2f, %.2f, %.2f] rad/s | "
               "dt: %.3f s\n",
               roll * (180.0 / M_PI), pitch * (180.0 / M_PI), yaw * (180.0 / M_PI),
               accel_raw[0], accel_raw[1], accel_raw[2],
               gyro_raw[0], gyro_raw[1], gyro_raw[2],
               dt);

    }

    return 0;
}