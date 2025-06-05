#include "API.h"
#include "common.h"
#include <math.h>
#include <vector>
#include <random>
#include <ctime>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

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
#define K_CTSTAN 1.0 // Stanley controller
#define K_SSTAN 0.05 

#define CELL_WIDTH 0.18

#define V_MAX 2.0
#define TURN_RADIUS (CELL_WIDTH / 2.0)
#define W_MAX (V_MAX / TURN_RADIUS)

class VController
{
public:
    double ePrev = 0;
    double eTot = 0;

    double Kp;
    double Ki;
    double Kd;

    // int tPrev;
    uint64_t tPrev;
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

        uint64_t t = time_us_64(); //[us]

        double dt = double(t - tPrev) / 1000000.0; // [s]

        printf("dt: %f Current time: %lld Previous time: %lld e: %f eTot: %f\n", dt, t, tPrev, e, eTot);

        // std::cout<<"dt: "<<dt<<"Current time: "<<t<<" Previous time: "<<tPrev<<std::endl;
        // std::cout<<"eTot: "<<eTot<<std::endl;
        eTot += e * dt;

        double output = Kp * e + Ki * eTot;

        this->tPrev = t;
        ePrev = e;

        return output;
    }

    void reset()
    {
        eTot = 0;
        tPrev = time_us_64();
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

    uint64_t tPrev;

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

        uint64_t t = time_us_64();

        double dt = double(t - tPrev) / 1000000;

        eTot += e * dt;

        double output = Kp * e + Ki * eTot;

        ePrev = e;
        tPrev = t;

        return int(output);
    }

    void reset()
    {
        eTot = 0;
        tPrev = time_us_64();
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

        return output;
    }
};

class StanleyController
{
public:
    double K_CT; // Cross-track gain
    double K_S;  // Smoothing gain
    double maxDelta;

    StanleyController(double K_CT, double K_S, double maxDelta)
    {
        this->K_CT = K_CT;
        this->K_S = K_S;
        this->maxDelta = maxDelta;
    }

    double output()
    {
        if (currentMovement == FWD)
        {
            double e = 0;

            if (targetPose.theta == 0)
            {
                e = POSE.x - targetPose.x; // cross-track error
            }
            else if (targetPose.theta == 90)
            {
                e = POSE.y - targetPose.y; // cross-track error
            }
            else if (targetPose.theta == 180)
            {
                e = targetPose.x - POSE.x; // cross-track error
            }
            else if (targetPose.theta == 270)
            {
                e = targetPose.y - POSE.y; // cross-track error
            }

            double psi = POSE.theta - targetPose.theta; // heading error

            double delta = psi + atan2(K_CT * e, POSE.v + K_S) * 180 / M_PI;

            double vL = targetPose.v * (1 - tan(delta * M_PI / 180) / 2);
            double vR = targetPose.v * (1 + tan(delta * M_PI / 180) / 2);
            double W = (vR - vL) / WHEEL_BASE;

            return W;
        }
        else if (currentMovement == TURN_L)
        {
            double xRoot, yRoot, xDesired, yDesired, phiCurrent;

            if (prevTargetPose.theta == 0)
            {
                double xRoot = targetPose.x; // center of rotation
                double yRoot = targetPose.y - TURN_RADIUS;
                double phiCurrent = atan2(POSE.y - yRoot, POSE.x - xRoot) * 180 / M_PI;
                double xDesired = xRoot + TURN_RADIUS * cos(phiCurrent * M_PI / 180);
                double yDesired = yRoot + TURN_RADIUS * sin(phiCurrent * M_PI / 180);
            }
            else if (prevTargetPose.theta == 90)
            {
                double xRoot = targetPose.x + TURN_RADIUS; // center of rotation
                double yRoot = targetPose.y;
                double phiCurrent = atan2(xRoot - POSE.x, POSE.y - yRoot) * 180 / M_PI;
                double xDesired = xRoot - TURN_RADIUS * sin(phiCurrent * M_PI / 180);
                double yDesired = yRoot + TURN_RADIUS * cos(phiCurrent * M_PI / 180);
            }
            else if (prevTargetPose.theta == 180)
            {
                double xRoot = targetPose.x;
                double yRoot = targetPose.y + TURN_RADIUS;
                double phiCurrent = atan2(yRoot - POSE.y, xRoot - POSE.x) * 180 / M_PI;
                double xDesired = xRoot - TURN_RADIUS * cos(phiCurrent * M_PI / 180);
                double yDesired = yRoot - TURN_RADIUS * sin(phiCurrent * M_PI / 180);
            }
            else if (prevTargetPose.theta == 270)
            {
                double xRoot = targetPose.x - TURN_RADIUS;
                double yRoot = targetPose.y;
                double phiCurrent = atan2(POSE.x - xRoot, yRoot - POSE.y) * 180 / M_PI;
                double xDesired = xRoot + TURN_RADIUS * sin(phiCurrent * M_PI / 180);
                double yDesired = yRoot - TURN_RADIUS * cos(phiCurrent * M_PI / 180);
            }

            double e = sqrt(pow(xRoot - POSE.x, 2) + pow(yRoot - POSE.y, 2)) - TURN_RADIUS;
            double thetaDesired = 90 - phiCurrent;
            double psi = POSE.theta - thetaDesired;

            double delta = psi + atan2(K_CT * e, POSE.v + K_S);

            double vL = targetPose.v * (1 - tan(delta * M_PI / 180) / 2);
            double vR = targetPose.v * (1 + tan(delta * M_PI / 180) / 2);
            double W = (vR - vL) / WHEEL_BASE;

            return W;
        }
        else if (currentMovement == TURN_R)
        {
            double xRoot, yRoot, xDesired, yDesired, phiCurrent;

            if (prevTargetPose.theta == 0)
            {
                double xRoot = targetPose.x; // center of rotation
                double yRoot = targetPose.y - TURN_RADIUS;
                double phiCurrent = atan2(POSE.y - yRoot, -POSE.x + xRoot) * 180 / M_PI;
                double xDesired = xRoot - TURN_RADIUS * cos(phiCurrent * M_PI / 180);
                double yDesired = yRoot + TURN_RADIUS * sin(phiCurrent * M_PI / 180);
            }
            else if (prevTargetPose.theta == 90)
            {
                double xRoot = targetPose.x + TURN_RADIUS; // center of rotation
                double yRoot = targetPose.y;
                double phiCurrent = atan2(xRoot - POSE.x, yRoot - POSE.y) * 180 / M_PI;
                double xDesired = xRoot - TURN_RADIUS * sin(phiCurrent * M_PI / 180);
                double yDesired = yRoot - TURN_RADIUS * cos(phiCurrent * M_PI / 180);
            }
            else if (prevTargetPose.theta == 180)
            {
                double xRoot = targetPose.x;
                double yRoot = targetPose.y + TURN_RADIUS;
                double phiCurrent = atan2(yRoot - POSE.y, POSE.x - xRoot)*180/M_PI;
                double xDesired = xRoot + TURN_RADIUS*cos(phiCurrent*M_PI/180);
                double yDesired = yRoot - TURN_RADIUS*sin(phiCurrent*M_PI/180);
                
            } else if (prevTargetPose.theta == 270) {
                double xRoot = targetPose.x - TURN_RADIUS;
                double yRoot = targetPose.y;
                double phiCurrent = atan2(POSE.x - xRoot, POSE.y - yRoot)*180/M_PI;
                double xDesired = xRoot + TURN_RADIUS*sin(phiCurrent*M_PI/180);
                double yDesired = yRoot + TURN_RADIUS*cos(phiCurrent*M_PI/180);
            }

            double e = sqrt(pow(xRoot - POSE.x, 2) + pow(yRoot - POSE.y, 2)) - TURN_RADIUS;
            double thetaDesired = 90 - phiCurrent;
            double psi = POSE.theta - thetaDesired;

            double delta = psi + atan2(K_CT * e, POSE.v + K_S);

            double vL = targetPose.v * (1 - tan(delta * M_PI / 180) / 2);
            double vR = targetPose.v * (1 + tan(delta * M_PI / 180) / 2);
            double W = (vR - vL) / WHEEL_BASE;

            return W;
        }   
        return 0;
    }
};

VController VContr(KP_V, KI_V, KD_V);
WController WContr(KP_W, KI_W, KD_W);
SteeringController SContr(KP_S, KI_S, KD_S);
StanleyController StanContr(K_CTSTAN, K_SSTAN, 0.0);

Motor MotorL(Motor_Choice::LEFT);
Motor MotorR(Motor_Choice::RIGHT);

using string = const char *;

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
    else {
        Motor.setPWM(0);
    }
}

std::pair<double, double> controlLoop(float vTarget, float wTarget)
{
    float rpmCurrentL = MotorL.readRPM();
    float rpmCurrentR = MotorR.readRPM();
    float vCurrentL = rpmCurrentL * WHEEL_RADIUS / 60;
    float vCurrentR = rpmCurrentR * WHEEL_RADIUS / 60;
    float vCurrentAVG = (vCurrentL + vCurrentR) / 2;
    float wCurrent = (vCurrentR - vCurrentL) / WHEEL_BASE;

    double vOut = VContr.output(vTarget, vCurrentAVG);
    double wOut = WContr.output(wTarget, wCurrent);
    double sOut = StanContr.output();
    wOut += sOut;

    printf("VOut: %f WOut: %f rpmL: %f rpmR: %f vL: %f vR: %f W: %f\n", vOut, wOut, rpmCurrentL, rpmCurrentR, vCurrentL, vCurrentR, wCurrent);

    double dutyL = std::max(std::min((vOut - wOut), 100.0), -100.0);
    double dutyR = std::max(std::min((vOut + wOut), 100.0), -100.0);

    return {dutyL, dutyR};
}

inline Pose getCurrentPose()
{
    // Get current pose
    printf("Current Pose: %f %f %f %f %f\n", POSE.x, POSE.y, POSE.theta, POSE.v, POSE.w);
    double x = POSE.x;
    double y = POSE.y;
    double theta = POSE.theta;

    // Read change in pos of each motor
    double posL = MotorL.readPOS() / 100;
    double posR = MotorR.readPOS() / 100;
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

void setTarget(Command command)
{
    prevTargetPose = targetPose;
    targetReached = false;
    if (command.action == "FWD")
    {
        // Forward for command.value cells
        currentMovement = FWD;
        targetPose.v = V_MAX;
        targetPose.w = 0;
        if (POSE.theta <= 0.1)
        {
            targetPose.y += CELL_WIDTH * command.value;
        }
        else if (POSE.theta - 90 <= 0.1)
        {
            targetPose.x -= CELL_WIDTH * command.value;
        }
        else if (POSE.theta - 180 <= 0.1)
        {
            targetPose.y -= CELL_WIDTH * command.value;
        }
        else if (POSE.theta - 270 <= 0.1)
        {
            targetPose.x += CELL_WIDTH * command.value;
        }
    }
    else if (command.action == "TRN" && command.value == 90)
    {
        currentMovement = TURN_L;
        targetPose.theta += 90;
        int theta = targetPose.theta;
        targetPose.theta = theta % 360;
        targetPose.v = V_MAX;
        targetPose.w = W_MAX;
        if (POSE.theta <= 0.1)
        {
            targetPose.x -= CELL_WIDTH / 2.0;
            targetPose.y += CELL_WIDTH / 2.0;
        }
        else if (POSE.theta - 90 <= 0.1)
        {
            targetPose.x -= CELL_WIDTH / 2.0;
            targetPose.y -= CELL_WIDTH / 2.0;
        }
        else if (POSE.theta - 180 <= 0.1)
        {
            targetPose.x += CELL_WIDTH / 2.0;
            targetPose.y -= CELL_WIDTH / 2.0;
        }
        else if (POSE.theta - 270 <= 0.1)
        {
            targetPose.x += CELL_WIDTH / 2.0;
            targetPose.y += CELL_WIDTH / 2.0;
        }
    }
    else if (command.action == "TRN" && command.value == -90)
    {
        currentMovement = TURN_R;
        targetPose.theta -= 90;
        int theta = targetPose.theta;
        targetPose.theta = theta % 360;
        targetPose.v = V_MAX;
        targetPose.w = -W_MAX;

        if (POSE.theta <= 0.1)
        {
            targetPose.x += CELL_WIDTH / 2.0;
            targetPose.y += CELL_WIDTH / 2.0;
        }
        else if (POSE.theta - 90 <= 0.1)
        {
            targetPose.x -= CELL_WIDTH / 2.0;
            targetPose.y += CELL_WIDTH / 2.0;
        }
        else if (POSE.theta - 180 <= 0.1)
        {
            targetPose.x -= CELL_WIDTH / 2.0;
            targetPose.y -= CELL_WIDTH / 2.0;
        }
        else if (POSE.theta - 270 <= 0.1)
        {
            targetPose.x += CELL_WIDTH / 2.0;
            targetPose.y -= CELL_WIDTH / 2.0;
        }
    }
    else if (command.action == "STOP") {
        currentMovement = STOP;
        targetPose.v = 0;
        targetPose.w = 0;
        if (POSE.theta <= 0.1)
        {
            targetPose.y += CELL_WIDTH;
        }
        else if (POSE.theta - 90 <= 0.1)
        {
            targetPose.x -= CELL_WIDTH;
        }
        else if (POSE.theta - 180 <= 0.1)
        {
            targetPose.y -= CELL_WIDTH;
        }
        else if (POSE.theta - 270 <= 0.1)
        {
            targetPose.x += CELL_WIDTH;
        }
    }
}

bool checkTargetReached()
{
    if (POSE - targetPose < Pose{0.1, 0.1, 0.1, 0.1, 0.1})
    {
        return true;
    }
    return false;
}


constexpr float DEFAULT_MAX_SENSOR_RANGE_MM = 255.0f;
constexpr float FRONT_MAX_SENSOR_RANGE_MM = 5.0f * 255.0f; // Front sensor has longer range

constexpr float EPSILON = 1e-9f;

struct Vec2f
{
    float x, y;

    Vec2f(float x_ = 0.0f, float y_ = 0.0f) : x(x_), y(y_) {}

    float norm() const
    {
        return sqrtf(x * x + y * y);
    }

    Vec2f normalized() const
    {
        float n = norm();
        if (n < EPSILON)
        {
            return Vec2f(0.0f, 0.0f);
        }
        return Vec2f(x / n, y / n);
    }

    Vec2f operator+(const Vec2f &other) const
    {
        return Vec2f(x + other.x, y + other.y);
    }

    Vec2f operator-(const Vec2f &other) const
    {
        return Vec2f(x - other.x, y - other.y);
    }

    Vec2f operator*(float scalar) const
    {
        return Vec2f(x * scalar, y * scalar);
    }

    friend Vec2f operator*(float scalar, const Vec2f &vec)
    {
        return vec * scalar;
    }
};

#define CELL_SIZE_MM 180.0f

constexpr float LOCAL_TOF_BASE_OFFSET_X_MM = 65.0f; // Offset along robot's local X-axis
constexpr float LOCAL_TOF_RADIAL_OFFSET_MM = 25.0f; // Additional radial offset for each sensor

constexpr float WALL_THICKNESS_MM = 12.0f; // Wall thickness in mm

struct RayMarchResult
{
    Vec2f point;
    bool hit_something; // True if wall or grid boundary hit, false if max range exceeded or invalid ray
};

RayMarchResult fast_ray_march_cpp(
    const Vec2f &ray_origin_world,
    const Vec2f &ray_dir_world,
    float max_sensor_range_mm)
{
    float norm_ray_dir = ray_dir_world.norm();
    if (norm_ray_dir < EPSILON)
    { // Ray direction is zero vector
        return {Vec2f(NAN, NAN), false};
    }
    Vec2f ray_dir_normalized = ray_dir_world * (1.0f / norm_ray_dir);

    const int maze_grid_width = MAZE_SIZE;
    const int maze_grid_height = MAZE_SIZE;

    int map_x = static_cast<int>(floorf(ray_origin_world.x / CELL_SIZE_MM));
    int map_y = static_cast<int>(floorf(ray_origin_world.y / CELL_SIZE_MM));

    int ray_step_x = (ray_dir_normalized.x > 0) ? 1 : ((ray_dir_normalized.x < 0) ? -1 : 0);
    int ray_step_y = (ray_dir_normalized.y > 0) ? 1 : ((ray_dir_normalized.y < 0) ? -1 : 0);

    float t_delta_x = (ray_step_x != 0) ? fabsf(CELL_SIZE_MM / ray_dir_normalized.x) : INFINITY;
    float t_delta_y = (ray_step_y != 0) ? fabsf(CELL_SIZE_MM / ray_dir_normalized.y) : INFINITY;

    float t_max_x, t_max_y;

    if (ray_step_x > 0)
    {
        t_max_x = ((static_cast<float>(map_x) + 1.0f) * CELL_SIZE_MM - ray_origin_world.x) / ray_dir_normalized.x;
    }
    else if (ray_step_x < 0)
    {
        t_max_x = (static_cast<float>(map_x) * CELL_SIZE_MM - ray_origin_world.x) / ray_dir_normalized.x;
    }
    else
    {
        t_max_x = INFINITY;
    }

    if (ray_step_y > 0)
    {
        t_max_y = ((static_cast<float>(map_y) + 1.0f) * CELL_SIZE_MM - ray_origin_world.y) / ray_dir_normalized.y;
    }
    else if (ray_step_y < 0)
    {
        t_max_y = (static_cast<float>(map_y) * CELL_SIZE_MM - ray_origin_world.y) / ray_dir_normalized.y;
    }
    else
    {
        t_max_y = INFINITY;
    }

    float current_distance_to_boundary = 0.0f;

    while (true)
    {
        bool cell_being_exited_was_valid = (map_x >= 0 && map_x < maze_grid_width) &&
                                           (map_y >= 0 && map_y < maze_grid_height);

        bool advancing_in_x;
        if (t_max_x < t_max_y)
        {
            current_distance_to_boundary = t_max_x;
            advancing_in_x = true;
        }
        else
        {
            current_distance_to_boundary = t_max_y;
            advancing_in_x = false;
        }

        if (current_distance_to_boundary > max_sensor_range_mm)
        {
            // Ray exceeded max range before hitting a wall or exiting grid within range
            return {ray_origin_world + ray_dir_normalized * max_sensor_range_mm, false};
        }

        if (cell_being_exited_was_valid)
        {
            int current_cell_y_idx = (MAZE_SIZE - 1) - map_y;

            // Ensure current_cell_y_idx is valid before accessing maze
            if (!(current_cell_y_idx >= 0 && current_cell_y_idx < maze_grid_height))
            {
                // This case implies ray started near boundary or an issue. Python returns True.
                return {ray_origin_world + ray_dir_normalized * current_distance_to_boundary, true};
            }

            Cell current_cell = MAZE_MATRIX[current_cell_y_idx][map_x];

            if (advancing_in_x)
            { // Stepping in X
                if (ray_step_x > 0)
                { // Moving Right (+X world)
                    if (current_cell.east)
                    { // Check cell's east wall
                        return {ray_origin_world + ray_dir_normalized * current_distance_to_boundary, true};
                    }
                }
                else if (ray_step_x < 0)
                { // Moving Left (-X world)
                    if (current_cell.west)
                    { // Check cell's west wall
                        return {ray_origin_world + ray_dir_normalized * current_distance_to_boundary + Vec2f(WALL_THICKNESS_MM, 0), true};
                    }
                }
            }
            else
            { // Stepping in Y
                if (ray_step_y > 0)
                { // Moving "Up" in world coords (+Y world)
                    if (current_cell.north)
                    { // Check cell's north wall
                        return {ray_origin_world + ray_dir_normalized * current_distance_to_boundary, true};
                    }
                }
                else if (ray_step_y < 0)
                { // Moving "Down" in world coords (-Y world)
                    if (current_cell.south)
                    { // Check cell's south wall
                        return {ray_origin_world + ray_dir_normalized * current_distance_to_boundary + Vec2f(0, WALL_THICKNESS_MM), true};
                    }
                }
            }
        }

        // Advance to next cell
        if (advancing_in_x)
        {
            map_x += ray_step_x;
            t_max_x += t_delta_x;
        }
        else
        {
            map_y += ray_step_y;
            t_max_y += t_delta_y;
        }

        // After stepping, if the new cell (map_x, map_y) is outside the grid,
        // the ray has exited. current_distance_to_boundary was the distance to this exit line.
        if (!((map_x >= 0 && map_x < MAZE_SIZE) && (map_y >= 0 && map_y < MAZE_SIZE)))
        {
            return {ray_origin_world + ray_dir_normalized * current_distance_to_boundary, true};
        }
    }
}

Vec2f rotate_vector_cpp(const Vec2f &v, float angle_rad)
{
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);
    return Vec2f(
        v.x * cos_a - v.y * sin_a,
        v.x * sin_a + v.y * cos_a);
}

const float SENSOR_ORIENTATIONS_RAD[NUM_TOF_SENSORS] = {
    0.0f,
    (float)M_PI / 2.0f,
    -(float)M_PI / 2.0f,
    (float)M_PI / 4.0f,
    -(float)M_PI / 4.0f};

Vec2f get_tof_location_cpp(const Vec2f &robot_pos_world, float robot_rot_rad, int sensor_index)
{
    Vec2f local_base_offset(LOCAL_TOF_BASE_OFFSET_X_MM, 0.0f);

    Vec2f radial_component_unrotated(LOCAL_TOF_RADIAL_OFFSET_MM, 0.0f);
    Vec2f sensor_specific_radial_offset = rotate_vector_cpp(radial_component_unrotated, SENSOR_ORIENTATIONS_RAD[sensor_index]);

    Vec2f total_local_offset = local_base_offset + sensor_specific_radial_offset;
    Vec2f offset_world = rotate_vector_cpp(total_local_offset, robot_rot_rad);
    Vec2f result = robot_pos_world + offset_world;
    return result;
}

constexpr float max_sensor_ranges_mm[] = {
    FRONT_MAX_SENSOR_RANGE_MM,
    DEFAULT_MAX_SENSOR_RANGE_MM,
    DEFAULT_MAX_SENSOR_RANGE_MM,
    DEFAULT_MAX_SENSOR_RANGE_MM,
    DEFAULT_MAX_SENSOR_RANGE_MM};

void update_tof_sensor_data_ray_marching_cpp(
    Vec2f noisy_sensor_points[NUM_TOF_SENSORS], // Output array for intersection points
    bool valid_readings[NUM_TOF_SENSORS],       // Output array for validity
    const Vec2f &robot_pos_world,
    float robot_rot_rad,
    float noise_factor = 0 // 0.005f
)
{
    // Initialize output arrays
    for (int i = 0; i < NUM_TOF_SENSORS; ++i)
    {
        noisy_sensor_points[i] = Vec2f(NAN, NAN);
        valid_readings[i] = false;
    }

    // Max sensor ranges for each sensor (mm)
    const float max_sensor_ranges_mm[NUM_TOF_SENSORS] = {
        FRONT_MAX_SENSOR_RANGE_MM,
        DEFAULT_MAX_SENSOR_RANGE_MM,
        DEFAULT_MAX_SENSOR_RANGE_MM,
        DEFAULT_MAX_SENSOR_RANGE_MM,
        DEFAULT_MAX_SENSOR_RANGE_MM};

    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < NUM_TOF_SENSORS; ++i)
    {
        // Get the actual origin of the TOF sensor's ray
        Vec2f tof_actual_origin_world = get_tof_location_cpp(robot_pos_world, robot_rot_rad, i);

        // Calculate the world angle of the sensor's ray
        // This uses the robot's orientation and the sensor's relative orientation
        float sensor_ray_world_angle_rad = robot_rot_rad + SENSOR_ORIENTATIONS_RAD[i];

        // Calculate ray direction vector in world frame
        Vec2f ray_dir_world(cosf(sensor_ray_world_angle_rad), sinf(sensor_ray_world_angle_rad));

        RayMarchResult result = fast_ray_march_cpp(
            tof_actual_origin_world, ray_dir_world, max_sensor_ranges_mm[i]);

        valid_readings[i] = result.hit_something;

        if (!isnan(result.point.x) && !isnan(result.point.y))
        { // Check if point is valid
            // Note: Python code calculates distance from robot_pos, not tof_actual_origin for noise.
            // This might be intentional if noise is related to overall estimate quality.
            // Let's follow Python: distance from robot_pos_world for noise calculation.
            Vec2f displacement_for_noise = result.point - robot_pos_world;
            float distance_for_noise = displacement_for_noise.norm();

            if (distance_for_noise > EPSILON)
            {
                float noise_std_dev = noise_factor * distance_for_noise;
                std::normal_distribution<float> dist(0.0f, noise_std_dev);
                Vec2f noise(dist(gen), dist(gen));
                noisy_sensor_points[i] = result.point + noise;
            }
            else
            {
                noisy_sensor_points[i] = result.point; // No noise if distance is (close to) zero
            }
        }
        else
        {
            // If fast_ray_march_cpp returned NaN (e.g. zero ray dir), point remains NaN.
            // valid_readings[i] would be false from result.hit_something.
            noisy_sensor_points[i] = result.point;
        }
    }
}

void print_maze()
{
    for (int r_idx = 0; r_idx < MAZE_SIZE; ++r_idx)
    {
        for (int c_idx = 0; c_idx < MAZE_SIZE; ++c_idx)
        {
            printf("+");
            printf("%s", (MAZE_MATRIX[r_idx][c_idx].north ? "---" : "   "));
        }
        printf("+\n");

        for (int c_idx = 0; c_idx < MAZE_SIZE; ++c_idx)
        {
            printf("%s", (MAZE_MATRIX[r_idx][c_idx].west ? "|" : " "));
            printf("   ");
        }
        printf("%s", (MAZE_MATRIX[r_idx][MAZE_SIZE - 1].east ? "|" : " "));
        printf("\n");
    }
    for (int c_idx = 0; c_idx < MAZE_SIZE; ++c_idx)
    {
        printf("+");
        printf("%s", (MAZE_MATRIX[MAZE_SIZE - 1][c_idx].south ? "---" : "   "));
    }
    printf("+\n");
}

// IMPORTANT: Ensure this string has consistent newlines ('\n') after each line of characters.
// The parser assumes a fixed stride including the newline.
const char *MAZE_ASCII_ART =
    "\n" // Initial newline, parser will skip this to match Python's strip() behavior
    "o---o---o---o---o---o---o---o---o---o---o---o---o---o---o---o---o\n"
    "|                               |                               |\n"
    "o   o---o---o---o---o---o---o   o   o---o---o---o   o   o---o   o\n"
    "|   |                                               |       |   |\n"
    "o   o   o   o---o---o---o---o---o---o---o---o---o   o---o   o---o\n"
    "|   |   |                                       |       |       |\n"
    "o   o   o   o---o---o---o---o---o---o---o---o   o---o   o---o   o\n"
    "|   |   |   |                               |       |       |   |\n"
    "o   o   o   o   o---o---o---o---o---o---o   o---o   o---o   o   o\n"
    "|   |   |   |   |                                               |\n"
    "o   o   o   o   o   o---o---o---o---o   o---o   o---o---o---o   o\n"
    "|   |   |   |       |               |       |       |           |\n"
    "o   o   o   o   o   o   o---o---o   o   o   o---o   o   o   o   o\n"
    "|   |   |   |   |       |       |       |       |       |   |   |\n"
    "o   o   o   o   o   o   o   o   o---o   o---o   o   o   o   o   o\n"
    "|       |   |   |   |   |   | S S S |       |   |   |   |       |\n" // Changed G G to S S S for testing
    "o---o   o   o   o   o   o   o   o   o   o   o   o   o   o   o---o\n"
    "|       |   |   |   |       | S S S |   |   |   |   |   |       |\n" // Changed G G to S S S for testing
    "o   o   o   o   o   o---o   o---o---o   o   o   o   o   o   o   o\n"
    "|   |   |       |       |               |   |   |   |   |   |   |\n"
    "o   o   o   o   o---o   o---o   o---o---o   o   o   o   o   o   o\n"
    "|   |   |   |       |       |               |   |   |   |   |   |\n"
    "o   o   o   o---o   o   o   o---o---o   o---o   o   o   o   o   o\n"
    "|   |   |       |       |                       |   |   |   |   |\n"
    "o---o   o---o---o---o   o---o---o---o---o---o---o   o   o   o   o\n"
    "|               |   |                               |   |   |   |\n"
    "o---o---o   o   o   o   o---o---o---o---o---o   o---o   o   o   o\n"
    "|   |       |   |   |                                   |   |   |\n"
    "o   o   o   o   o   o---o---o---o---o---o---o---o   o---o   o   o\n"
    "|       |       |                               |           |   |\n"
    "o   o---o---o   o---o---o---o   o   o---o---o---o---o   o---o   o\n"
    "| S                             |                               |\n" // Start S
    "o---o---o---o---o---o---o---o---o---o---o---o---o---o---o---o---o";

void parse_maze_string(const char *maze_str_input)
{
    const char *current_pos = maze_str_input;

    // Skip a single leading newline if present (to mimic Python's .strip() on a string starting with \n)
    if (current_pos[0] == '\n')
    {
        current_pos++;
    }

    const int chars_in_string_line = MAZE_SIZE * 4 + 1; // e.g., "o---o---o" for MAZE_SIZE=2 is 2*4+1 = 9 chars
    const int string_stride = chars_in_string_line + 1; // +1 for the newline character '\n'

    for (int cell_r = 0; cell_r < MAZE_SIZE; ++cell_r)
    {
        for (int cell_c = 0; cell_c < MAZE_SIZE; ++cell_c)
        {
            // Initialize cell: no walls, not visited.
            MAZE_MATRIX[cell_r][cell_c].walls = 0;

            // Determine character positions in the 1D string `current_pos`
            // North Wall: on text line `cell_r * 2`, at character `cell_c * 4 + 2` within that line.
            int north_wall_text_line_idx = cell_r * 2;
            int north_wall_char_idx_in_line = cell_c * 4 + 2;
            char north_char = current_pos[north_wall_text_line_idx * string_stride + north_wall_char_idx_in_line];
            if (north_char == '-')
            {
                MAZE_MATRIX[cell_r][cell_c].north = WALL;
            }

            // South Wall: on text line `(cell_r * 2) + 2`, at character `cell_c * 4 + 2` within that line.
            int south_wall_text_line_idx = cell_r * 2 + 2;
            int south_wall_char_idx_in_line = cell_c * 4 + 2;
            char south_char = current_pos[south_wall_text_line_idx * string_stride + south_wall_char_idx_in_line];
            if (south_char == '-')
            {
                MAZE_MATRIX[cell_r][cell_c].south = WALL;
            }

            // West Wall: on text line `(cell_r * 2) + 1`, at character `cell_c * 4` within that line.
            int west_wall_text_line_idx = cell_r * 2 + 1;
            int west_wall_char_idx_in_line = cell_c * 4;
            char west_char = current_pos[west_wall_text_line_idx * string_stride + west_wall_char_idx_in_line];
            if (west_char == '|')
            {
                MAZE_MATRIX[cell_r][cell_c].west = WALL;
            }

            // East Wall: on text line `(cell_r * 2) + 1`, at character `(cell_c * 4) + 4` within that line.
            int east_wall_text_line_idx = cell_r * 2 + 1;
            int east_wall_char_idx_in_line = cell_c * 4 + 4;
            char east_char = current_pos[east_wall_text_line_idx * string_stride + east_wall_char_idx_in_line];
            if (east_char == '|')
            {
                MAZE_MATRIX[cell_r][cell_c].east = WALL;
            }

            // Visited is not set by this parser, defaults to 0.
            MAZE_MATRIX[cell_r][cell_c].visited = 0;
        }
    }
}

#define NUM_PARTICLES 500

struct Particle
{
    Vec2f pos;
    float rot_rad;
    float weight;
};

const float SENSOR_NOISE_STDDEV = 90.0f; // mm, tune this!
const float SENSOR_NOISE_VAR = SENSOR_NOISE_STDDEV * SENSOR_NOISE_STDDEV;

Particle particles_a[NUM_PARTICLES] = {};
Particle particles_b[NUM_PARTICLES] = {};

Particle *particles = particles_a;     // Pointer to current particle set
Particle *new_particles = particles_b; // Pointer to new particle set

float rand_gauss(float mean, float stddev)
{
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return z0 * stddev + mean;
}

void localize_particles()
{
    // Read actual TOF measurements
    global_read_tofs();

    float weight_sum = 0.0f;
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle &p = particles[i];

        Vec2f sim_points[NUM_TOF_SENSORS];
        bool sim_valid[NUM_TOF_SENSORS];

        update_tof_sensor_data_ray_marching_cpp(sim_points, sim_valid, p.pos, p.rot_rad);

        float total_error = 0.0f;
        for (int i = 0; i < NUM_TOF_SENSORS; ++i)
        {
            if (!MM_VALID[i] || !sim_valid[i])
                continue;

            Vec2f sim_origin = get_tof_location_cpp(p.pos, p.rot_rad, i);
            float sim_dist = (sim_points[i] - sim_origin).norm();
            float real_dist = static_cast<float>(MM[i]);

            float diff = real_dist - sim_dist;
            total_error += diff * diff;
        }

        // Likelihood using Gaussian model
        p.weight = expf(-total_error / SENSOR_NOISE_VAR);
        weight_sum += p.weight;
    }

    // Normalize weights
    if (weight_sum <= 1e-6f)
    {
        printf("WARNING: Total weight is zero! Resetting weights.\n");
        for (int i = 0; i < NUM_PARTICLES; ++i)
        {
            particles[i].weight = 1.0f / NUM_PARTICLES;
        }
    }
    else
    {
        for (int i = 0; i < NUM_PARTICLES; ++i)
        {
            particles[i].weight /= weight_sum;
        }
    }

    // Low-variance resampling
    float r = ((float)rand() / RAND_MAX) / NUM_PARTICLES;
    float c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; ++m)
    {
        float U = r + m * (1.0f / NUM_PARTICLES);
        while (U > c && i < NUM_PARTICLES - 1)
        {
            ++i;
            c += particles[i].weight;
        }
        new_particles[m] = particles[i];

        new_particles[m].pos.x += rand_gauss(0.0f, 3.0f); // mm
        new_particles[m].pos.y += rand_gauss(0.0f, 3.0f); // mm
        new_particles[m].rot_rad += rand_gauss(0.0f, 0.05f); // radians
    }

    std::swap(particles, new_particles);
}

float random_angle()
{
    // Generate a random angle in radians between 0 and 2*PI
    return static_cast<float>(rand()) / RAND_MAX * 2.0f * (float)M_PI;
}

Vec2f random_free_cell_center()
{
    int x = rand() % (MAZE_SIZE + 1);
    int y = rand() % (MAZE_SIZE + 1);

    return Vec2f((x + 0.5f) * CELL_SIZE_MM, (y + 0.5f) * CELL_SIZE_MM);
}

Vec2f sample_around_start_point()
{
    // Sample a random point around the start point (0, 0) within a certain radius
    float radius = rand() % (int)(CELL_SIZE_MM * 2.0f); // Random radius up to 2 cells
    float angle = random_angle();
    float x = radius * cosf(angle);
    float y = radius * sinf(angle);
    return Vec2f(x, y);
}

void motion_update(float dx, float dy, float drot)
{
    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        float noise_x = rand_gauss(0.0f, 5.0f); // mm
        float noise_y = rand_gauss(0.0f, 5.0f); // mm
        float noise_rot = rand_gauss(0.0f, 0.05f); // radians

        particles[i].pos.x += dx + noise_x;
        particles[i].pos.y += dy + noise_y;
        particles[i].rot_rad += drot + noise_rot;
    }
}

#define ROBOT_WHEEL_BASE_MM 80.0f

int main()
{
    srand(time(NULL)); // Seed the random number generator

    parse_maze_string(MAZE_ASCII_ART);
    print_maze();

    global_init();

    Vec2f robot_pos(CELL_SIZE_MM * 0.5f, CELL_SIZE_MM * 0.5f);
    float robot_rot_rad = (float)M_PI / 2.0f;

    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        particles[i].pos = sample_around_start_point(); // your own method
        particles[i].rot_rad = random_angle();        // e.g., float between 0 and 2*PI
        particles[i].weight = 1.0f / NUM_PARTICLES;
    }

#if 0
    printf("Robot position: (%.2f, %.2f) mm\n", robot_pos.x, robot_pos.y);
    printf("Robot rotation: %.2f degrees\n", (robot_rot_rad * 180.0f / (float)M_PI));

    Vec2f sensor_points_data[NUM_TOF_SENSORS];
    bool sensor_valid_readings[NUM_TOF_SENSORS];

    update_tof_sensor_data_ray_marching_cpp(
        sensor_points_data, sensor_valid_readings,
        robot_pos, robot_rot_rad);

    printf("\nSimulated Noisy Sensor Intersection Points (mm) & Validity:\n");
    const char *sensor_names[NUM_TOF_SENSORS] = {"FRONT        ", "LEFT         ", "RIGHT        ", "FRONT_LEFT_45", "FRONT_RIGHT_45"};
    for (int i = 0; i < NUM_TOF_SENSORS; ++i)
    {
        printf("%s: ", sensor_names[i]);
        if (isnan(sensor_points_data[i].x))
        {
            printf("(NaN, NaN)");
        }
        else
        {
            // Distance from actual TOF origin to hit point
            Vec2f tof_origin = get_tof_location_cpp(robot_pos, robot_rot_rad, i);
            float dist_from_tof_origin = (sensor_points_data[i] - tof_origin).norm();
            if (sensor_valid_readings[i])
            {
                printf("(%.2f, %.2f)  Dist: %.2f mm \n",
                       sensor_points_data[i].x, sensor_points_data[i].y, dist_from_tof_origin);
            }
            else
            {
                printf("(Invalid) \n");
            }
        }
    }

    global_read_tofs();

    printf("\nActual TOF Sensor Readings (mm):\n");
    for (int i = 0; i < NUM_TOF_SENSORS; ++i)
    {
        printf("%s: ", sensor_names[i]);
        if (MM_VALID[i])
        {
            printf("Approx Dist: %d mm\n", MM[i]);
        }
        else
        {
            printf("(Invalid)\n");
        }
    }

    Vec2f ray_origin = Vec2f(CELL_SIZE_MM * 0.5f, CELL_SIZE_MM * 0.5f);
    Vec2f ray_dir = Vec2f(1.0f, 0.0f);
    printf("\nDirect ray march from (%.1f,%.1f) pointing right:\n", ray_origin.x, ray_origin.y);
    RayMarchResult hit_result = fast_ray_march_cpp(ray_origin, ray_dir, DEFAULT_MAX_SENSOR_RANGE_MM * 2);
    if (isnan(hit_result.point.x))
    {
        printf("Hit: (NaN, NaN)");
    }
    else
    {
        printf("Hit: (%.2f, %.2f)", hit_result.point.x, hit_result.point.y);
    }
    printf("  Hit Something: %s\n", hit_result.hit_something ? "true" : "false");
#endif

    uint64_t tPrev = time_us_64();
    while (true)
    {        
        global_read_tofs();
        global_read_imu();
     
        uint64_t tNow = time_us_64();
        float dt = (tNow - tPrev) / 1e6f; // Convert microseconds to seconds
        tPrev = tNow;

        float dx_left = MotorL.readPOS();
        float dx_right = MotorR.readPOS();

        float cos_heading = cosf(robot_rot_rad);
        float sin_heading = sinf(robot_rot_rad);
        float dx_world = (dx_left * cos_heading + dx_right * cos_heading) / 2.0f; // Average of left and right wheel displacements
        float dy_world = (dx_left * sin_heading + dx_right * sin_heading) / 2.0f;

        printf("dx_left: %.2f mm, dx_right: %.2f mm\n", dx_left, dx_right);

        float rps = GYRO_Z * (float)M_PI / 180.0f; // Convert degrees per second to radians per second
        float drot_rad = rps * dt; // Change in rotation in radians
        motion_update(dx_world, dy_world, drot_rad);

        printf(">>> vizPARTICLES ");
        for (int i = 0; i < NUM_PARTICLES; ++i)
        {
            printf("%.2f %.2f %.2f ", particles[i].pos.x, particles[i].pos.y, particles[i].rot_rad);
        }
        printf("\n");
        fflush(stdout);

        localize_particles();

        sleep_ms(100);
    }

    return 0;

    double vtarg = 5.0;
    double wtarg = 0.0;
    VContr.reset();
    double vout1 = VContr.output(vtarg, 0.0);
    double vout2 = VContr.output(vtarg, 2.0);
    double vout3 = VContr.output(vtarg, 3.0);
    setTarget(Command {"FWD", 3});
    printf("Current Pose: %f %f %f %f %f\nTarget Pose: %f %f %f %f %f\nPrevious Target Pose: %f %f %f %f %f\n", POSE.x, POSE.y, POSE.theta, POSE.v, POSE.w, targetPose.x, targetPose.y, targetPose.theta, targetPose.v, targetPose.w, prevTargetPose.x, prevTargetPose.y, prevTargetPose.theta, prevTargetPose.v, prevTargetPose.w);

    // POSE = {-0.02, 0.02, 15, 0.95*V_MAX, W_MAX};
    // double w = StanContr.output();

    // printf("W: %f", w);

    while (true) {
        auto [dutyL, dutyR] = controlLoop(5, 0);

        MotorL.setPWM((float) dutyL);
        MotorR.setPWM((float) dutyR);

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

    // Queue<Command> commandQueue = {Command {"STOP", 0}, Command {"FWD", 3}};
    // setTarget(commandQueue.pop());
    // // commandQueue.push(Command {"FWD", 0})

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
            }*/
        }
        else
        {
            sleep_ms(100);
        }

        // POSE = getCurrentPose();
        // auto [dutyL, dutyR] = controlLoop(targetPose.v, targetPose.w);
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
