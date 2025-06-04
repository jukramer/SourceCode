#include "API.h"
#include "common.h"
#include <math.h>
#include <vector>

// Controller Constants
#define KP_V 200.0 // Velocity controller
#define KI_V 40.0
#define KD_V 5.32089706902006
#define KP_W 50.0 // Angular velocity controller
#define KI_W 5.0
#define KD_W 0.238664893739785
#define KP_S 1.0 // Steering controller
#define KI_S 1.0
#define KD_S 0.0

#define V_MAX 2.0
#define W_MAX 3.0

#define CELL_WIDTH 0.18

class VController
{
public:
    double ePrev = 0;
    double eTot = 0;

    double Kp;
    double Ki;
    double Kd;

    // int tPrev;
    int64_t tPrev;
    // int64_t tPrev;

    VController(double Kp, double Ki, double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        tPrev = time_us_64();
    }

    int output(double vRef, double vCurrent)
    {
        double e = vRef - vCurrent;
        int t = time_us_64();

        double dt = (t - tPrev) / 1000000.0;

        // std::cout<<"dt: "<<dt<<"Current time: "<<t<<" Previous time: "<<tPrev<<std::endl;
        // std::cout<<"eTot: "<<eTot<<std::endl;
        eTot += e * dt;

        double output = Kp * e + Ki * eTot + Kd * (e - ePrev) / dt;
        tPrev = t;

        ePrev = e;
        return int(output);
    }
};

class WController
{
public:
    double ePrev = 0;
    double eTot = 0;

    double Kp;
    double Ki;
    double Kd;

    int64_t tPrev;

    WController(double Kp, double Ki, double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        tPrev = time_us_64();
    }

    int output(double wRef, double wCurrent)
    {
        double e = wRef - wCurrent;
        int t = time_us_64();
        double dt = (t - tPrev) / 1000000;
        eTot += e * dt;
        double output = Kp * e + Ki * eTot;
        ePrev = e;
        return int(output);
    }
};

class SteeringController
{
public:
    double ePrev = 0;
    double eTot = 0;
    double tPrev;

    double Kp;
    double Ki;
    double Kd;

    SteeringController(double Kp, double Ki, double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        tPrev = time_us_64();
    }

    int output(double diffTOF)
    {
        int t = time_us_64();
        double dt = (t - tPrev) / 1000000;

        eTot += diffTOF * dt;

        double output = Kp * diffTOF + Ki * eTot;
        ePrev = diffTOF;

        return int(output);
    }
};

VController VContr(KP_V, KI_V, KD_V);
WController WContr(KP_W, KI_W, KD_W);
SteeringController SContr(KP_S, KI_S, KD_S);

Motor MotorL(Motor_Choice::LEFT);
Motor MotorR(Motor_Choice::RIGHT);

class StanleyController
{
public:
    StanleyController()
    {
    }
};

using string = const char *;

template <typename T, size_t N = 512>
class Array
{
private:
    T buffer[N];
    size_t size_ = 0;

public:
    void push_back(const T &val)
    {
        if (size_ < N)
        {
            buffer[size_++] = val;
        }
        else
        {
            printf("Out of space");
        }
    }
    T &operator[](size_t idx) { return buffer[idx]; }
    size_t size() const { return size_; }
};

class Trajectory
{
public:
    Trajectory(Array<string> &commands)
    {
        for (int i = 0; i < commands.size(); i++)
        {
            string command = commands[i];
        }
    }

    /*StatePrediction getPos()
    {
    }*/
};

void motorTest(Motor &Motor)
{
    if (stdio_usb_connected())
    {
        printf("Forward...\n");
        Motor.setPWM(255);
        sleep_ms(100);
        int rpm1 = Motor.readRPM();
        sleep_ms(50);
        int rpm2 = Motor.readRPM();
        sleep_ms(500);
        Motor.readRPM();
        sleep_ms(10);
        int rpm3 = Motor.readRPM();
        printf("RPM1: %d RPM2: %d RPM3: %d\n", rpm1, rpm2, rpm3);
        sleep_ms(300);
        printf("Stopping...\n");
        Motor.setPWM(0);
        sleep_ms(1000);

        printf("Backward...\n");
        Motor.setPWM(-255);
        sleep_ms(300);

        printf("Stopping...\n");
        Motor.setPWM(0);
        sleep_ms(1000);
    }
    else
    {
        Motor.setPWM(0);
    }
}

std::pair<int, int> controlLoop(float vTarget, float wTarget)
{
    float rpmCurrentL = MotorL.readRPM();
    float rpmCurrentR = MotorR.readRPM();
    float vCurrentL = rpmCurrentL * WHEEL_RADIUS / 60;
    float vCurrentR = rpmCurrentR * WHEEL_RADIUS / 60;
    float vCurrentAVG = (vCurrentL + vCurrentR) / 2;
    float wCurrent = (vCurrentR - vCurrentL) / WHEEL_BASE;

    double vOut = VContr.output(vTarget, vCurrentAVG);
    double wOut = WContr.output(wTarget, wCurrent);

    printf("VOut: %f WOut: %f rpmL: %f rpmR: %f vL: %f vR: %f W: %f\n", vOut, wOut, rpmCurrentL, rpmCurrentR, vCurrentL, vCurrentR, wCurrent);

    int dutyL = int(std::max(std::min((vOut - wOut), 100.0), -100.0));
    int dutyR = int(std::max(std::min((vOut + wOut), 100.0), -100.0));

    return {dutyL, dutyR};
}

inline Pose getCurrentPose() {
    // Get current pose
    printf("Current Pose: %f %f %f %f %f\n", POSE.x, POSE.y, POSE.theta, POSE.v, POSE.w);
    double x = POSE.x;
    double y = POSE.y;
    double theta = POSE.theta;

    // Read change in pos of each motor
    double posL = MotorL.readPOS()/100;
    double posR = MotorR.readPOS()/100;
    double dPos = (posL + posR)/2;
    double dTheta = (posR - posL)/WHEEL_BASE;

    printf("Pos updates: dPos: %f dTheta: %f\n", dPos, dTheta);

    // Update pose
    x += dPos*cos(POSE.theta);
    y += dPos*cos(POSE.theta);
    theta += dTheta;
    Pose newPose = {x, y, theta, POSE.v, POSE.w};

    printf("New pose: %f %f %f %f %f\n", newPose.x, newPose.y, newPose.theta, newPose.v, newPose.w);

    return newPose;
}

void setTarget(Command command) {
    targetReached = false;
    if (command.action == "FWD") {
        currentMovement = FWD;
        targetPose.v = V_MAX;
        targetPose.w = 0;
        if (POSE.theta <= 0.1) {
            targetPose.y += CELL_WIDTH*command.value;
        } else if (POSE.theta - 90 <= 0.1) {
            targetPose.x -= CELL_WIDTH*command.value;
        } else if (POSE.theta - 180 <= 0.1) {
            targetPose.y -= CELL_WIDTH*command.value;
        } else if (POSE.theta - 270 <= 0.1) {
            targetPose.x += CELL_WIDTH*command.value;
        }
    }

    if (command.action == "STOP") {
        currentMovement = STOP;
        targetPose.v = 0;
        targetPose.w = 0;
        if (POSE.theta <= 0.1) {
            targetPose.y += CELL_WIDTH;
        } else if (POSE.theta - 90 <= 0.1) {
            targetPose.x -= CELL_WIDTH;
        } else if (POSE.theta - 180 <= 0.1) {
            targetPose.y -= CELL_WIDTH;
        } else if (POSE.theta - 270 <= 0.1) {
            targetPose.x += CELL_WIDTH;
        }
    }
    // } else if (step == 'L') {
    //     currentMovement = TURN_L;
    //     if (POSE.theta <= 0.1) {
    //         targetPose.y += CELL_WIDTH;
    //     } else if (POSE.theta - 90 <= 0.1) {
    //         targetPose.x -= CELL_WIDTH;
    //     } else if (POSE.theta - 180 <= 0.1) {
    //         targetPose.y -= CELL_WIDTH;
    //     } else if (POSE.theta - 270 <= 0.1) {
    //         targetPose.x += CELL_WIDTH;
    //     }
    // }
}

bool checkTargetReached() {
    if (POSE-targetPose < Pose {0.1, 0.1, 0.1, 0.1, 0.1}) {
        return true;
    }
    return false;
}


int main()
{
    global_init();

    double vtarg = 5;
    double vout1 = VContr.output(vtarg, 0);
    double vout2 = VContr.output(vtarg, 2);
    double vout3 = VContr.output(vtarg, 3);

    double wout1 = WContr.output(vtarg, 0);
    double wout2 = WContr.output(vtarg, 2);
    double wout3 = WContr.output(vtarg, 3);
    // std::vector<Command> commands = stateMachineSimple("FLFLS");

    Queue<Command> commandQueue = {Command {"STOP", 0}, Command {"FWD", 3}};
    setTarget(commandQueue.pop());
    // commandQueue.push(Command {"FWD", 0})

    while (true)
    {
        

        // auto reading = global_get_tof(TOF_Direction::FRONT);
        // printf("TOF Front: %f Valid: %d\n", reading.distance, (int)reading.valid);

        // sleep_ms(100);

        if (stdio_usb_connected())
        {
            global_read_tofs();
            continue;

            ;
            /*std::vector<Command> commands = stateMachineSimple("FFLRFF");

            for (Command command : commands)
            {
                printf("%s %f ", command.action, command.value);
            }*/
        } else {
            sleep_ms(100);
        }

        POSE = getCurrentPose();
        auto [dutyL, dutyR] = controlLoop(targetPose.v, targetPose.w);  
        MotorL.setPWM(dutyL);
        MotorR.setPWM(dutyR);

        targetReached = checkTargetReached();

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
}

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
           roll * (180.0 / M_PI), pitch * (180.0 / M_PI), yaw * (180.0 / M_PI),
           accel_raw[0], accel_raw[1], accel_raw[2],
           gyro_raw[0], gyro_raw[1], gyro_raw[2],
           dt);

}
*/
