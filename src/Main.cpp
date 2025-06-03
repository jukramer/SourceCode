#include "API.h"

// Controller Constants
#define KP_V 200.0 // Velocity controller
#define KI_V 40.0
#define KD_V 5.32089706902006
#define KP_W 50.0 // Angular velocity controller
#define KI_W 5.0
#define KD_W 0.238664893739785

#define V_MAX 2.0
#define W_MAX 3.0

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

    double output(double vRef, double vCurrent)
    {
        double e = vRef - vCurrent;
        int t = time_us_64();

        double dt = (t - tPrev)/1000000.0;

        // std::cout<<"dt: "<<dt<<"Current time: "<<t<<" Previous time: "<<tPrev<<std::endl;
        // std::cout<<"eTot: "<<eTot<<std::endl; 
        eTot += e * dt;
        
        double output = Kp * e + Ki * eTot + Kd * (e - ePrev)/dt;
        tPrev = t;

        ePrev = e;
        return output;
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

    double output(double wRef, double wCurrent)
    {
        double e = wRef - wCurrent;
        int t = time_us_64();
        double dt = (t - tPrev) / 1000000;
        eTot += e * dt;
        double output = Kp * e + Ki * eTot;
        ePrev = e;
        return output;
    }
};

void motorTest(Motor &Motor) {
    if (stdio_usb_connected()) {
        printf("Forward...\n");
        Motor.setPWM(255, FORWARD);
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
        Motor.setPWM(0, BACKWARD);
        sleep_ms(1000);

        printf("Backward...\n");
        Motor.setPWM(255, BACKWARD);
        sleep_ms(300);
        
        printf("Stopping...\n");
        Motor.setPWM(0, BACKWARD);
        sleep_ms(1000);
    } else {
        Motor.setPWM(0, BACKWARD);
    }
}


std::pair<int, int> controlLoop(VController &VContr, WController &WContr, Mouse &mouse, Motor &MotorL, Motor &MotorR, float vTarget, float wTarget) {
    float rpmCurrentL = MotorL.readRPM();
    float rpmCurrentR = MotorR.readRPM();
    float vCurrentL = rpmCurrentL*WHEEL_RADIUS/60;
    float vCurrentR = rpmCurrentR*WHEEL_RADIUS/60;
    float vCurrentAVG = (vCurrentL + vCurrentR)/2;
    float wCurrent = (vCurrentR - vCurrentL)/WHEEL_BASE;

    double vOut = VContr.output(vTarget, vCurrentAVG);
    double wOut = WContr.output(wTarget, wCurrent);

    printf("VOut: %f WOut: %f rpmL: %f rpmR: %f vL: %f vR: %f W: %f\n", vOut, wOut, rpmCurrentL, rpmCurrentR, vCurrentL, vCurrentR, wCurrent);

    int dutyL = int(std::max(std::min((vOut - wOut), 220.0), 0.0));
    int dutyR = int(std::max(std::min((vOut + wOut), 220.0), 0.0));

    return {dutyL, dutyR};
}


int main()
{        
    API::init();

    VController VContr(KP_V, KI_V, KD_V);
    WController WContr(KP_W, KI_W, KD_W);

    Motor MotorL(dirA1Pin, dirA2Pin, spdAPin, mrencPin, &totalTicksL, &c2_callback);
    Motor MotorR(dirB1Pin, dirB2Pin, spdBPin, mlencPin, &totalTicksR, &c1_callback);

    double vtarg = 5;
    double vout1 = VContr.output(vtarg, 0);
    double vout2 = VContr.output(vtarg, 2);
    double vout3 = VContr.output(vtarg, 3);

    double wout1 = WContr.output(vtarg, 0);
    double wout2 = WContr.output(vtarg, 2);
    double wout3 = WContr.output(vtarg, 3);

    while (true) {
        if (stdio_usb_connected()) {
            auto [pwmL, pwmR] = controlLoop(VContr, WContr, mouse, MotorL, MotorR, 2, 0);
            MotorL.setPWM(pwmL, FORWARD);
            MotorR.setPWM(pwmR, FORWARD);
            printf("Duty Left: %d Duty Right: %d\n", pwmL, pwmR);
            
        } else {
            MotorL.setPWM(0, FORWARD);
            MotorR.setPWM(0, FORWARD);
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
    }

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
    */

