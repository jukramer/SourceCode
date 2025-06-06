#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

#define I2C_FREQ 100000

typedef struct {
    i2c_inst_t *i2c;
    uint8_t sda;
    uint8_t scl;
} i2c_pin_pair;

// i2c_pin_pair i2c_pin_pairs[] = {
//     {i2c0, 0, 1}, {i2c0, 4, 5}, {i2c0, 8, 9}, {i2c0, 12, 13}, {i2c0, 16, 17}, {i2c0, 20, 21},
//     {i2c1, 2, 3}, {i2c1, 6, 7}, {i2c1, 10, 11}, {i2c1, 14, 15}, {i2c1, 18, 19}, {i2c1, 26, 27}
// };

i2c_pin_pair i2c_pin_pairs[] = {
    // {i2c0, 1, 2}, {i2c0, 6, 7}, {i2c0, 11, 12}, {i2c0, 16, 17}, {i2c0, 21, 22}, {i2c0, 26, 27},
    // {i2c1, 4, 5}, {i2c1, 9, 10}, {i2c1, 14, 15}, {i2c1, 19, 20}, {i2c1, 24, 25}, {i2c1, 31, 32}
    {i2c0, 14, 15}, {i2c1, 14, 15}, {i2c0, 15, 14}, {i2c1, 15, 14}
};

bool scan_address(i2c_inst_t *i2c, uint8_t addr) {
    uint8_t buf;
    int ret = i2c_read_timeout_us(i2c, addr, &buf, 1, false, 2000);
    return ret >= 0;
}

int main() {
    stdio_init_all();

    gpio_init(14);
    gpio_init(15);
    gpio_init(17);
    gpio_set_dir(17, GPIO_OUT);
    gpio_put(17, 1);
    
    while (!stdio_usb_connected()) {
        sleep_ms(1000);
    }
    printf("Starting I2C scan across all valid pin pairs...\n");

    for (int i = 0; i < sizeof(i2c_pin_pairs)/sizeof(i2c_pin_pair); ++i) {
        i2c_pin_pair pair = i2c_pin_pairs[i];
        printf("\nTrying I2C%d on SDA=%d, SCL=%d\n", pair.i2c == i2c0 ? 0 : 1, pair.sda, pair.scl);

        i2c_deinit(pair.i2c);  // Reset in case it was already used
        i2c_init(pair.i2c, I2C_FREQ);
        gpio_set_function(pair.sda, GPIO_FUNC_I2C);
        gpio_set_function(pair.scl, GPIO_FUNC_I2C);
        gpio_pull_up(pair.sda);
        gpio_pull_up(pair.scl);

        sleep_ms(500);

        for (uint8_t addr = 0x00; addr <= 0x77; ++addr) {
            // printf("  -> 0x%02X\n", addr);
            if (scan_address(pair.i2c, addr)) {
                printf("  -> Found device at 0x%02X\n", addr);
            }
        }
    }

    printf("\nScan complete.\n");
    return 0;
}


/*

    float vtarg = 5.0;
    float wtarg = 0.0;
    VContr.reset();
    float vout1 = VContr.output(vtarg, 0.0);
    float vout2 = VContr.output(vtarg, 2.0);
    float vout3 = VContr.output(vtarg, 3.0);
    setTarget(Command{"FWD", 3});
    printf("Current Pose: %f %f %f %f %f\nTarget Pose: %f %f %f %f %f\nPrevious Target Pose: %f %f %f %f %f\n", POSE.x, POSE.y, POSE.theta, POSE.v, POSE.w, TARGET_POSE.x, TARGET_POSE.y, TARGET_POSE.theta, TARGET_POSE.v, TARGET_POSE.w, prevTARGET_POSE.x, prevTARGET_POSE.y, prevTARGET_POSE.theta, prevTARGET_POSE.v, prevTARGET_POSE.w);

    // POSE = {-0.02, 0.02, 15, 0.95*V_MAX, W_MAX};
    // double w = StanContr.output();

    // printf("W: %f", w);

    while (true)
    {
        auto [dutyL, dutyR] = controlLoop(5, 0);

        MotorL.setPWM((float)dutyL);
        MotorR.setPWM((float)dutyR);

        printf("%f %f\n", dutyL, dutyR);
    }

    // double vtarg = 5.0;
    // double wtarg = 0.0;
    // VContr.reset();
    // double vout1 = VContr.output(vtarg, 0.0);
    // double vout2 = VContr.output(vtarg, 2.0);
    // double vout3 = VContr.output(vtarg, 3.0);

    // double wout1 = WContr.output(wtarg, -1);
    // double wout2 = WContr.output(wtarg, -0.2);
    // double wout3 = WContr.output(wtarg, 0);
    // printf("%f %f %f %f %f %f\n", vout1, vout2, vout3, wout1, wout2, wout3);
    // std::vector<Command> commands = stateMachineSimple("FLFLS");



    while (true)
    {
        sleep_ms(100);
        continue;

        // auto reading = global_get_tof(TOF_Direction::FRONT);
        // printf("TOF Front: %f Valid: %d\n", reading.distance, (int)reading.valid);

        // sleep_ms(100);

        if (stdio_usb_connected())
        {
            // global_read_tofs();
            // global_read_imu();
            continue;

            ;
            /*std::vector<Command> commands = stateMachineSimple("FFLRFF");

            for (Command command : commands)
            {
                printf("%s %f ", command.action, command.value);
            }
        }
        else
        {
            sleep_ms(100);
        }


inline Pose getCurrentPose()
{
    // Get current pose
    printf("Current Pose: %f %f %f %f %f\n", POSE.x, POSE.y, POSE.theta, POSE.v, POSE.w);
    double x = POSE.x;
    double y = POSE.y;
    double theta = POSE.theta;

    // Read change in pos of each motor
    double posL = MotorL.DELTA_POS / 100;
    double posR = MotorR.DELTA_POS / 100;
    double dPos = (posL + posR) / 2;
    double dTheta = (posR - posL) / WHEEL_BASE;

    printf("Pos updates: dPos: %f dTheta: %f\n", dPos, dTheta);

    // Update pose
    x += dPos * cos(POSE.theta);
    y += dPos * cos(POSE.theta);
    theta += dTheta;
    Pose newPose = {x, y, theta, POSE.v, POSE.w};

    printf("New pose: %f %f %f %f %f\n", newPose.x, newPose.y, newPose.theta, newPose.v, newPose.w);

    return newPose;
}

        // POSE = getCurrentPose();
        // auto [dutyL, dutyR] = controlLoop(TARGET_POSE.v, TARGET_POSE.w);
        // MotorL.setPWM(dutyL);
        // MotorR.setPWM(dutyR);

        // // Set new target if old one reached
        // targetReached = checkTargetReached();
        // if (targetReached) {
        //     setTarget(commandQueue.pop());
        // }

        // if (stdio_usb_connected())
        // {
        //     ;
        //     // for (Command command : commands) {
        //     //     printf("%s %d ", command.action, command.value);
        //     //     printf("\n");
        //     // }

        //     // auto [pwmL, pwmR] = controlLoop(VContr, WContr, MotorL, MotorR, 2, 0);
        //     // MotorL.setPWM(pwmL);
        //     // MotorR.setPWM(pwmR);
        //     // printf("Duty Left: %d Duty Right: %d\n", pwmL, pwmR);
        // }
        // else
        // {
        //     // MotorL.setPWM(0);
        //     // MotorR.setPWM(0);
        //     ;
        // // }
        // }
    }
*/
// printf("VOut1: %f VOut2: %f VOut3: %f\nWOut1: %f WOut2: %f WOut3: %f", vout1, vout2, vout3, wout1, wout2, wout3);

// while (true) {
//     // motorTest(MotorL);
//     // motorTest(MotorR);
//     if(stdio_usb_connected()) {
//         // motorTest(MotorR);
//         // motorTest(MotorL);
//         MotorL.setPWM(255, FORWARD);
//         MotorR.setPWM(255, FORWARD);
//         sleep_ms(100);
//         printf("RPM: %f, %f", MotorL.readRPM(), MotorR.readRPM());
//     } else {
//         MotorR.setPWM(0, FORWARD);
//         MotorL.setPWM(0, FORWARD);
//     }

// double dutyL, dutyR = controlLoop(VContr, WContr, mouse, MotorL, MotorR, 1000, 0);
// double rpmL = MotorL.readRPM();
// double rpmR = MotorR.readRPM();
// printf("Duty L: %f Duty R: %f\n", dutyL, dutyR);
// printf("RPM Left: %f RPM Right: %f\n", rpmL, rpmR);
// MotorL.setPWM(dutyL, FORWARD);
// MotorR.setPWM(dutyR, FORWARD);

// gpio_put(dirA1Pin, 1);
// gpio_put(dirA2Pin, 0);
// analogWrite(spdAPin, 200);

// double out = VContr.output(1000, 0);

/*

// Global State Machine
while (true)
{
    if (mouse.state == INIT)
    {
        // stdio_init_all();
        // for (int i = 0; i < 27; i++){
        //     gpio_init(allPins[i]);
        //     gpio_set_dir(allPins[i], GPIO_OUT);
        // }
        mouse.state = IDLE;
    }
    else if (mouse.state == IDLE)
    {
        mouse.state = MAP_FLOODFILL;
    }
    else if (mouse.state == MAP_FLOODFILL || mouse.state == MAP_FLOODFILL_BACK)
    {
        scanWalls(mouse);
        floodFill(mouse);

        Direction minFloodFillDirection;
        int minFloodFill = 255;

        int x = mouse.cellPos.x;
        int y = mouse.cellPos.y;

        uint8_t mask = moveMask[y][x];
        for (int i = 0; i < 4; ++i) {
            if (!(mask & (1 << i))) continue;

            int nx = x + DIRECTIONS[i].x;
            int ny = y + DIRECTIONS[i].y;

            if (floodMatrix[ny][nx] <= minFloodFill)
            {
                minFloodFill = floodMatrix[ny][nx];
                minFloodFillDirection = (Direction)i;
            }
        }

        // std::cout << "Flood fill direction: " << minFloodFillDirection << std::endl;
        // std::cout << "Flood fill value: " << minFloodFill << std::endl;

        mouse.move(minFloodFillDirection);

        if (floodMatrix[mouse.cellPos.y][mouse.cellPos.x] == 0)
        {
            if (mouse.state == MAP_FLOODFILL)
            {
                std::cout << "Reached goal! Backing..." << std::endl;
                mouse.state = MAP_FLOODFILL_BACK;
            } else if (mouse.state == MAP_FLOODFILL_BACK)
            {
                if (mouse.cellPos.x == 0 && mouse.cellPos.y == MAZE_SIZE - 1)
                {
                    std::cout << "Reached start! Stopping..." << std::endl;
                    break;
                }
            }
        }
    }
}

    for (int y = 0; y < MAZE_SIZE; y++)
{
    for (int x = 0; x < MAZE_SIZE; x++)
    {
        uint8_t mask = 0b1111;

        if (y == 0)
            mask &= ~(1 << TOP);
        if (x == MAZE_SIZE - 1)
            mask &= ~(1 << RIGHT);
        if (y == MAZE_SIZE - 1)
            mask &= ~(1 << BOTTOM);
        if (x == 0)
            mask &= ~(1 << LEFT);

        moveMask[y][x] = mask;
    }
}

 //
// Main loop
//



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
           roll * (180.0 / PI), pitch * (180.0 / PI), yaw * (180.0 / PI),
           accel_raw[0], accel_raw[1], accel_raw[2],
           gyro_raw[0], gyro_raw[1], gyro_raw[2],
           dt);

}
*/
