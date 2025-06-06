#include "API.h"
#include "common.h"
#include <string>
#include <vector>
#include <random>
#include <time.h>
#include "linalg.h"
#include <math.h>

#define PI 3.14159265358979323846f // Define PI constant

// Controller Constants
#define KP_V 2.0f // Velocity controller (tuned for mm/s)
#define KI_V 0.4f // (tuned for mm/s)
// KD_V not used in VController currently
#define KP_W 50.0f // Angular velocity controller (rad/s)
#define KI_W 5.0f  // (rad/s)
// KD_W not used in WController currently
#define K_CTSTAN 1.0f // Stanley controller cross-track gain
#define K_SSTAN 5.0f  // Stanley controller smoothing gain (mm/s)

// Maze and Robot Dimensions (all in millimeters or radians)
#define CELL_WIDTH 180.0f                   // mm
#define V_MAX 500.0f                        // mm/s (Reduced V_MAX for more stable testing initially)
#define TURN_RADIUS (CELL_WIDTH / 2.0f)     // mm
#define W_MAX_NOMINAL (V_MAX / TURN_RADIUS) // Nominal angular velocity for turns (rad/s)

#define SENSOR_NOISE_STDDEV 0.1f
const float SENSOR_NOISE_VAR = SENSOR_NOISE_STDDEV * SENSOR_NOISE_STDDEV;

constexpr float DEFAULT_MAX_SENSOR_RANGE_MM = 255.0f;
constexpr float FRONT_MAX_SENSOR_RANGE_MM = 10.0f * 255.0f; // Front sensor has longer range

#define CELL_SIZE_MM 180.0f

constexpr float LOCAL_TOF_BASE_OFFSET_X_MM = 65.0f; // Offset along robot's local X-axis
constexpr float LOCAL_TOF_RADIAL_OFFSET_MM = 25.0f; // Additional radial offset for each sensor

constexpr float WALL_THICKNESS_MM = 12.0f; // Wall thickness in mm

struct Particle
{
    Vec2f pos;     // mm
    float rot_rad; // radians, [-PI, PI), 0 along +X, PI/2 along +Y
    float weight;
};

float normalize_angle_pi_pi(float angle_rad)
{
    angle_rad = fmodf(angle_rad + PI, 2.0f * PI);
    if (angle_rad < 0.0f)
    {
        angle_rad += 2.0f * PI;
    }
    return angle_rad - PI;
}

Pose POSE(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
int CELL_X = 0;
int CELL_Y = 0;

Pose targetPose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
Pose prevTargetPose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
MovementType currentMovement = IDLE;
bool targetReached = true;

class VController
{
public:
    float ePrev = 0;
    float eTot = 0;
    float Kp, Ki, Kd; // Kd not used
    uint64_t tPrev;

    VController(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd)
    {
        tPrev = time_us_64();
    }

    float output(float vRef_mmps, float vCurrent_mmps)
    {
        float e = vRef_mmps - vCurrent_mmps;
        uint64_t t = time_us_64();
        float dt = static_cast<float>(t - tPrev) / 1000000.0f;

        if (dt >= EPSILON)
        {
            eTot += e * dt;
        }
        // Optional: Clamp eTot
        // eTot = std::max(-MAX_ETOT_V, std::min(MAX_ETOT_V, eTot));

        float out_val = Kp * e + Ki * eTot; // Kd term: + Kd * (e - ePrev) / dt;

        tPrev = t;
        ePrev = e;
        tPrev = t;
        ePrev = e;
        return out_val;
    }

    void reset()
    {
        eTot = 0;
        ePrev = 0;
        tPrev = time_us_64();
    }
};


class WController
{
public:
    float ePrev = 0;
    float eTot = 0;
    float Kp, Ki, Kd; // Kd not used
    uint64_t tPrev;

    WController(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd)
    {
        tPrev = time_us_64();
    }

    float output(float wRef_radps, float wCurrent_radps)
    {
        float e = wRef_radps - wCurrent_radps; // Error in rad/s
        uint64_t t = time_us_64();
        float dt = static_cast<float>(t - tPrev) / 1000000.0f;

        if (dt >= EPSILON)
        {
            eTot += e * dt;
        }
        // Optional: Clamp eTot
        // eTot = std::max(-MAX_ETOT_W, std::min(MAX_ETOT_W, eTot));

        float out_val = Kp * e + Ki * eTot;

        tPrev = t;
        ePrev = e;
        return out_val;
    }

    void reset()
    {
        eTot = 0;
        ePrev = 0;
        tPrev = time_us_64();
    }
};

class StanleyController
{
public:
    float K_CT; // Cross-track gain
    float K_S;  // Smoothing gain for effective velocity (v_e = POSE.v_mmps + K_S)

    StanleyController(float k_ct, float k_s) : K_CT(k_ct), K_S(k_s) {}

    // Output is desired angular velocity (rad/s)
    float output()
    {
        // POSE values are in mm, radians, mm/s, rad/s
        // targetPose values are in mm, radians, mm/s, rad/s

        float desired_w_radps = 0.0f;

        if (currentMovement == FWD)
        {
            // Cross-track error e_ct (mm)
            // Path is a line defined by (targetPose.x, targetPose.y) and angle targetPose.theta
            // e_ct = (POSE.x - targetPose.x) * sin(targetPose.theta) - (POSE.y - targetPose.y) * cos(targetPose.theta)
            // This formula gives signed distance. Positive if POSE is to the "left" of directed line targetPose,
            // assuming standard math angles (0 along +X).
            float dx = POSE.x - targetPose.x; // Vector from target point on path to robot
            float dy = POSE.y - targetPose.y;
            float path_angle_rad = targetPose.theta; // Robot should follow this orientation

            // Cross-track error: perpendicular distance to the line.
            // The line passes through targetPose.x, targetPose.y with angle targetPose.theta.
            // A point on this line is (targetPose.x, targetPose.y).
            // A normal vector to the path (pointing left) is (-sin(path_angle_rad), cos(path_angle_rad)).
            // e_ct is the projection of (dx, dy) onto this normal.
            float e_ct_mm = dx * (-sinf(path_angle_rad)) + dy * cosf(path_angle_rad);
            // Simpler way if targetPose is the "destination" and robot moves along path_angle_rad:
            // If path_angle_rad is 0 (+X axis): e_ct = POSE.y - targetPose.y
            // If path_angle_rad is PI/2 (+Y axis): e_ct = -(POSE.x - targetPose.x) = targetPose.x - POSE.x
            // Let's use the general line distance formula for clarity, assuming targetPose.x/y is a point on the desired path.
            // The line is (targetPose.x, targetPose.y) going in direction targetPose.theta.
            // Error relative to path line x*sin(th_p) - y*cos(th_p) - (x_p*sin(th_p) - y_p*cos(th_p)) = 0
            e_ct_mm = (POSE.x - targetPose.x) * sinf(targetPose.theta) - (POSE.y - targetPose.y) * cosf(targetPose.theta);

            // Heading error psi (radians)
            float psi_rad = normalize_angle_pi_pi(POSE.theta - targetPose.theta);

            // Effective velocity for atan2 term
            float v_effective_mmps = fabsf(POSE.v) + K_S;
            if (v_effective_mmps < 10.0f)
                v_effective_mmps = 10.0f; // Avoid issues at very low speed (e.g. 10mm/s)

            // Steering angle delta (radians)
            float delta_rad = psi_rad + atan2f(K_CT * e_ct_mm, v_effective_mmps);
            delta_rad = normalize_angle_pi_pi(delta_rad); // Keep delta reasonable

            // Convert steering angle delta to angular velocity W = v * tan(delta) / L
            if (fabsf(targetPose.v) < EPSILON)
            {
                desired_w_radps = K_CT * psi_rad; // If not moving, just correct heading slowly
            }
            else
            {
                // Clamp delta_rad to avoid extreme tan values, e.g., +/- PI/3 (60 deg)
                delta_rad = std::max(-PI / 3.0f, std::min(PI / 3.0f, delta_rad));
                desired_w_radps = (targetPose.v / WHEEL_BASE_MM) * tanf(delta_rad);
            }
            // printf("Stanley FWD: e_ct:%.1fmm psi:%.2frad delta:%.2frad -> W_des:%.2frad/s\n", e_ct_mm, psi_rad, delta_rad, desired_w_radps);
        }
        else if (currentMovement == TURN_L || currentMovement == TURN_R)
        {
            float xRoot_mm = 0, yRoot_mm = 0; // Center of the turn circle (mm)
            bool root_defined = false;

            // prevTargetPose.theta is orientation (rad, 0=+X) before starting this turn command.
            // prevTargetPose.x,y is position (mm) where turn initiated.
            float prev_th_rad = normalize_angle_pi_pi(prevTargetPose.theta);

            // Angle comparisons (e.g., if prev_th_rad is close to 0, PI/2, etc.)
            // 0 rad: +X, PI/2 rad: +Y, PI rad: -X, -PI/2 rad: -Y
            if (currentMovement == TURN_L)
            { // Turning CCW
                if (fabsf(prev_th_rad - 0.0f) < 0.1f)
                { // Was going +X, turning Left (towards +Y)
                    xRoot_mm = prevTargetPose.x;
                    yRoot_mm = prevTargetPose.y + TURN_RADIUS;
                    root_defined = true;
                }
                else if (fabsf(prev_th_rad - (PI / 2.0f)) < 0.1f)
                { // Was going +Y, turning Left (towards -X)
                    xRoot_mm = prevTargetPose.x - TURN_RADIUS;
                    yRoot_mm = prevTargetPose.y;
                    root_defined = true;
                }
                else if (fabsf(fabsf(prev_th_rad) - PI) < 0.1f)
                { // Was going -X (theta ~ PI or -PI)
                    xRoot_mm = prevTargetPose.x;
                    yRoot_mm = prevTargetPose.y - TURN_RADIUS;
                    root_defined = true;
                }
                else if (fabsf(prev_th_rad - (-PI / 2.0f)) < 0.1f)
                { // Was going -Y
                    xRoot_mm = prevTargetPose.x + TURN_RADIUS;
                    yRoot_mm = prevTargetPose.y;
                    root_defined = true;
                }
            }
            else
            { // TURN_R (Turning CW)
                if (fabsf(prev_th_rad - 0.0f) < 0.1f)
                { // Was going +X, turning Right (towards -Y)
                    xRoot_mm = prevTargetPose.x;
                    yRoot_mm = prevTargetPose.y - TURN_RADIUS;
                    root_defined = true;
                }
                else if (fabsf(prev_th_rad - (PI / 2.0f)) < 0.1f)
                { // Was going +Y, turning Right (towards +X)
                    xRoot_mm = prevTargetPose.x + TURN_RADIUS;
                    yRoot_mm = prevTargetPose.y;
                    root_defined = true;
                }
                else if (fabsf(fabsf(prev_th_rad) - PI) < 0.1f)
                { // Was going -X
                    xRoot_mm = prevTargetPose.x;
                    yRoot_mm = prevTargetPose.y + TURN_RADIUS;
                    root_defined = true;
                }
                else if (fabsf(prev_th_rad - (-PI / 2.0f)) < 0.1f)
                { // Was going -Y
                    xRoot_mm = prevTargetPose.x - TURN_RADIUS;
                    yRoot_mm = prevTargetPose.y;
                    root_defined = true;
                }
            }

            if (!root_defined)
            {
                // printf("Stanley TURN: ERROR - Could not determine turn center. PrevTh: %.2frad\n", prev_th_rad);
                return 0.0f;
            }

            // Cross-track error for circular path (mm)
            float e_ct_mm = sqrtf(powf(POSE.x - xRoot_mm, 2) + powf(POSE.y - yRoot_mm, 2)) - TURN_RADIUS;

            // Desired heading (tangent to circle)
            float angle_to_pose_rad = atan2f(POSE.y - yRoot_mm, POSE.x - xRoot_mm);
            float desired_heading_rad;
            if (currentMovement == TURN_L)
            {
                desired_heading_rad = angle_to_pose_rad + PI / 2.0f;
            }
            else
            { // TURN_R
                desired_heading_rad = angle_to_pose_rad - PI / 2.0f;
            }
            desired_heading_rad = normalize_angle_pi_pi(desired_heading_rad);

            // Heading error psi (rad)
            float psi_rad = normalize_angle_pi_pi(POSE.theta - desired_heading_rad);
            // Alternative: psi error w.r.t final target orientation of the turn.
            // float psi_rad = normalize_angle_pi_pi(POSE.theta - targetPose.theta);

            float v_effective_mmps = fabsf(POSE.v) + K_S;
            if (v_effective_mmps < 10.0f)
                v_effective_mmps = 10.0f;

            float delta_rad = psi_rad + atan2f(K_CT * e_ct_mm, v_effective_mmps);
            delta_rad = normalize_angle_pi_pi(delta_rad);
            delta_rad = std::max(-PI / 3.0f, std::min(PI / 3.0f, delta_rad)); // Clamp steering

            if (fabsf(targetPose.v) < EPSILON)
            {
                desired_w_radps = 0.0f;
            }
            else
            {
                desired_w_radps = (targetPose.v / WHEEL_BASE_MM) * tanf(delta_rad);
                // Cap based on nominal turn rate
                float cap = W_MAX_NOMINAL * 1.2f; // Allow some overshoot from nominal
                if (desired_w_radps > cap)
                    desired_w_radps = cap;
                if (desired_w_radps < -cap)
                    desired_w_radps = -cap;
            }
            // printf("Stanley TURN: e_ct:%.1fmm psi:%.2frad delta:%.2frad -> W_des:%.2frad/s\n", e_ct_mm, psi_rad, delta_rad, desired_w_radps);
        }
        else
        { // STOP_CMD or IDLE
            desired_w_radps = 0.0f;
        }
        return desired_w_radps;
    }
};

VController VContr(KP_V, KI_V, 0.0f /*KD_V not used*/);
WController WContr(KP_W, KI_W, 0.0f /*KD_W not used*/);
StanleyController StanContr(K_CTSTAN, K_SSTAN);

Motor MotorL(Motor_Choice::LEFT);
Motor MotorR(Motor_Choice::RIGHT);

using string = const char *;




void motorTest(Motor &Motor)
{
    if (stdio_usb_connected())
    {
        printf("Forward...\n");
        Motor.setPWM(100);
        sleep_ms(100);
        Motor.update();
        float rpm1 = Motor.RPM;
        sleep_ms(50);
        Motor.update();
        float rpm2 = Motor.RPM;
        sleep_ms(500);
        Motor.update();
        float rpm3 = Motor.RPM;
        printf("Forward RPM1: %f RPM2: %f RPM3: %f\n", rpm1, rpm2, rpm3);

        sleep_ms(300);
        printf("Stopping...\n");
        Motor.setPWM(0);
        sleep_ms(1000);

        printf("Backward...\n");
        Motor.setPWM(-100);
        sleep_ms(100);
        Motor.update();
        rpm1 = Motor.RPM;
        sleep_ms(50);
        Motor.update();
        rpm2 = Motor.RPM;
        sleep_ms(500);
        Motor.update();
        rpm3 = Motor.RPM;
        printf("Backward RPM1: %f RPM2: %f RPM3: %f\n", rpm1, rpm2, rpm3);

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

std::pair<float, float> controlLoop(float vTarget, float wTarget)
{
    float rpmCurrentL = MotorL.RPM;
    float rpmCurrentR = MotorR.RPM;
    float vCurrentL = rpmCurrentL * WHEEL_RADIUS_MM / 60;
    float vCurrentR = rpmCurrentR * WHEEL_RADIUS_MM / 60;
    float vCurrentAVG = (vCurrentL + vCurrentR) / 2;
    POSE.v = (double)vCurrentAVG;
    float wCurrent = (vCurrentR - vCurrentL) / WHEEL_BASE_MM;
    POSE.w = (double)wCurrent;

    float vOut = VContr.output(vTarget, vCurrentAVG);
    float wOut = WContr.output(wTarget, wCurrent);
    float sOut = StanContr.output();
    wOut += sOut;

    printf("VOut: %f WOut: %f rpmL: %f rpmR: %f vL: %f vR: %f W: %f\n", vOut, wOut, rpmCurrentL, rpmCurrentR, vCurrentL, vCurrentR, wCurrent);

    float dutyL = std::max(std::min((vOut - wOut), 100.0f), -100.0f);
    float dutyR = std::max(std::min((vOut + wOut), 100.0f), -100.0f);

    return {dutyL, dutyR};
}

Direction theta_to_direction()
{
    float theta = POSE.theta;

    // Normalize to [-PI, PI)
    while (theta >= PI)
        theta -= 2.0f * PI;
    while (theta < -PI)
        theta += 2.0f * PI;

    if (theta > -PI / 4 && theta <= PI / 4)
        return Direction::RIGHT; // +X
    else if (theta > PI / 4 && theta <= 3 * PI / 4)
        return Direction::TOP; // +Y
    else if (theta <= -PI / 4 && theta > -3 * PI / 4)
        return Direction::BOTTOM; // -Y
    else
        return Direction::LEFT; // -X
}

void setTarget(Command command)
{
    prevTargetPose = targetPose; // The target we just arrived at (or current POSE if first command)
    targetReached = false;

    if (command.action == "FWD")
    {
        currentMovement = FWD;

        Direction dir = theta_to_direction();
        printf("FWD command: %s, value: %.2f, dir: %d\n", command.action.c_str(), command.value, dir);
        int dir_index = static_cast<int>(dir);

        int cells_forward = static_cast<int>(command.value);

        int target_cell_x = CELL_X + OFFSET_LOCATIONS[dir_index].x * cells_forward;
        int target_cell_y = CELL_Y - OFFSET_LOCATIONS[dir_index].y * cells_forward;

        targetPose.theta = POSE.theta;
        targetPose.v = V_MAX;
        targetPose.w = 0.0f;

        // Calculate wall-aligned stop point
        targetPose.x = target_cell_x * CELL_WIDTH + (dir == Direction::LEFT ? CELL_WIDTH : 0);
        targetPose.y = target_cell_y * CELL_WIDTH + (dir == Direction::BOTTOM ? CELL_WIDTH : 0);

        // Compensate for sensor offset
        if (dir == Direction::RIGHT) {
            //targetPose.x += LOCAL_TOF_BASE_OFFSET_X_MM;
            targetPose.y += CELL_WIDTH / 2.0f; // Center in cell
        }
        if (dir == Direction::LEFT) {
            //targetPose.x -= LOCAL_TOF_BASE_OFFSET_X_MM;
            targetPose.y += CELL_WIDTH / 2.0f; // Center in cell
        }
        if (dir == Direction::TOP) {
            targetPose.x += CELL_WIDTH / 2.0f; // Center in cell
            //targetPose.y += LOCAL_TOF_BASE_OFFSET_X_MM / 2;
        }
        if (dir == Direction::BOTTOM) {
            targetPose.x += CELL_WIDTH / 2.0f; // Center in cell
            //targetPose.y -= LOCAL_TOF_BASE_OFFSET_X_MM;
        }
    }
    else if (command.action == "TRN")
    {
        float turn_angle_rad = command.value; // radians, positive for left (CCW)
        if (turn_angle_rad > 0)
            currentMovement = TURN_L;
        else
            currentMovement = TURN_R;

        currentMovement = (turn_angle_rad > 0) ? TURN_L : TURN_R;

        targetPose.theta = normalize_angle_pi_pi(POSE.theta + turn_angle_rad);
        targetPose.v = V_MAX;
        targetPose.w = (turn_angle_rad > 0 ? W_MAX_NOMINAL : -W_MAX_NOMINAL) * 0.75f;

        // Center of current cell, nudged by half a cell in turn direction
        Direction dir = theta_to_direction();
        int dir_index = static_cast<int>(dir);

        float offset = CELL_WIDTH / 2.0f;
        targetPose.x = CELL_X * CELL_WIDTH + CELL_WIDTH / 2.0f + OFFSET_VECTORS[dir_index].x * offset;
        targetPose.y = CELL_Y * CELL_WIDTH + CELL_WIDTH / 2.0f + OFFSET_VECTORS[dir_index].y * offset;
    }
    else if (command.action == "STOP")
    {
        currentMovement = STOP_CMD;
        targetPose.v = 0.0f;
        targetPose.w = 0.0f;
        targetPose.x = POSE.x;
        targetPose.y = POSE.y;
        targetPose.theta = POSE.theta;
        targetReached = true;
    }
}

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
    (float)PI / 2.0f,
    -(float)PI / 2.0f,
    (float)PI / 4.0f,
    -(float)PI / 4.0f};

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

std::random_device random_device;
std::mt19937 rand_gen(random_device());

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
                Vec2f noise(dist(rand_gen), dist(rand_gen));
                noisy_sensor_points[i] = result.point; // + noise;
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
    "o   o   o---o---o---o   o---o---o---o---o---o---o   o   o   o   o\n"
    "|               |   |                               |   |   |   |\n"
    "o   o---o   o   o   o   o---o---o---o---o---o   o---o   o   o   o\n"
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

#define NUM_PARTICLES 3000

Particle particles_a[NUM_PARTICLES] = {};
Particle particles_b[NUM_PARTICLES] = {};

Particle *particles = particles_a;     // Pointer to current particle set
Particle *new_particles = particles_b; // Pointer to new particle set

float rand_gauss(float mean, float stddev)
{
    std::normal_distribution<float> dist(mean, stddev);
    return dist(rand_gen);
}

float rand_uniform(float min, float max)
{
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rand_gen);
}

float random_angle()
{
    // Generate a random angle in radians between 0 and 2*PI
    return rand_uniform(0.0f, 2.0f * (float)PI);
}

Vec2f sample_around_pose()
{
    float radius = rand_uniform(0.0f, 100.0f);
    float angle = random_angle();
    float x = POSE.x + radius * cosf(angle);
    float y = POSE.y + radius * sinf(angle);
    return Vec2f(x, y);
}

void resample_particles()
{
    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        particles[i].pos = sample_around_pose();
        particles[i].rot_rad = POSE.theta + rand_uniform(-0.3f, 0.3f);
        particles[i].weight = 1.0f / NUM_PARTICLES;
    }
}

void localize_particles()
{
    float weight_sum = 0.0f;
    int valid_sensor_count = 0;
    for (int i = 0; i < NUM_PARTICLES; i++)
    {
        Particle &p = particles[i];

        Vec2f sim_points[NUM_TOF_SENSORS];
        bool sim_valid[NUM_TOF_SENSORS];

        update_tof_sensor_data_ray_marching_cpp(sim_points, sim_valid, p.pos, p.rot_rad);

        float total_error = 0.0f;
        for (int j = 0; j < NUM_TOF_SENSORS; ++j)
        {
            if (!MM_VALID[j] || !sim_valid[j])
                continue;

            Vec2f sim_origin = get_tof_location_cpp(p.pos, p.rot_rad, j);
            float sim_dist = (sim_points[j] - sim_origin).norm();
            float real_dist = static_cast<float>(MM[j]);

            float diff = real_dist - sim_dist;
            // printf("Particle %d Sensor %d: Real: %.1fmm, Simulated: %.1fmm, Diff: %.1fmm\n", i, j, real_dist, sim_dist, diff);
            total_error += diff * diff;
            valid_sensor_count++;
        }

        float sigma_sq = SENSOR_NOISE_VAR;
        float avg_error = total_error / (float)(valid_sensor_count + 1e-5f);
        p.weight = fmaxf(expf(-avg_error / (2.0f * sigma_sq)), 1e-30f);

        weight_sum += p.weight;
    }

    // Normalize weights
    if (weight_sum <= 1e-6f)
    {
        printf("WARNING: Total weight is zero! Resetting weights.\n");
        resample_particles();
    }
    else
    {
        for (int i = 0; i < NUM_PARTICLES; ++i)
        {
            particles[i].weight /= weight_sum;
        }
    }

    // Low-variance resampling
    float r = rand_uniform(0.0f, 1.0f) / NUM_PARTICLES;
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

        new_particles[m].pos.x += rand_gauss(0.0f, 2.0f);    // mm
        new_particles[m].pos.y += rand_gauss(0.0f, 2.0f);    // mm
        new_particles[m].rot_rad += rand_gauss(0.0f, 0.02f); // radians
    }
    std::swap(particles, new_particles);
}

Vec2f random_free_cell_center()
{
    int x = (int)rand_uniform(0, MAZE_SIZE); // Random cell index in [0, MAZE_SIZE)
    int y = (int)rand_uniform(0, MAZE_SIZE); // Random cell index in [0, MAZE_SIZE)

    return Vec2f((x + 0.5f) * CELL_SIZE_MM, (y + 0.5f) * CELL_SIZE_MM);
}

void motion_update(float dx, float dy, float drot)
{
    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        float x = particles[i].pos.x;
        float y = particles[i].pos.y;

        float nx = x + dx;
        float ny = y + dy;

        int old_cell_x = (int)(x / CELL_SIZE_MM);
        int old_cell_y = 15 - (int)(y / CELL_SIZE_MM);

        int cell_x = (int)(nx / CELL_SIZE_MM);
        int cell_y = 15 - (int)(ny / CELL_SIZE_MM);

        if (old_cell_x != cell_x || old_cell_y != cell_y)
        {
            // Check if the new cell is valid
            if (cell_x < 0 || cell_x >= MAZE_SIZE || cell_y < 0 || cell_y >= MAZE_SIZE)
            {
                // Out of bounds, reset to old position
                nx = x;
                ny = y;
            }
            else
            {
                Direction dir;
                if (cell_x > old_cell_x)
                    dir = Direction::LEFT;
                else if (cell_x < old_cell_x)
                    dir = Direction::RIGHT;
                else if (cell_y > old_cell_y)
                    dir = Direction::TOP;
                else if (cell_y < old_cell_y)
                    dir = Direction::BOTTOM;

                Cell &new_cell = MAZE_MATRIX[cell_y][cell_x];
                if (new_cell.walls & (1 << (int)dir))
                {
                    // Wall encountered — revert
                    nx = x;
                    ny = y;
                }
            }
        }

        float noise_x = rand_gauss(0.0f, 1.0f);    // mm
        float noise_y = rand_gauss(0.0f, 1.0f);    // mm
        float noise_rot = rand_gauss(0.0f, 0.01f); // radians

        particles[i].pos.x = nx + noise_x;
        particles[i].pos.y = ny + noise_y;
        particles[i].rot_rad = normalize_angle_pi_pi(particles[i].rot_rad + drot + noise_rot);
    }
}

#define ROBOT_WHEEL_BASE_MM 80.0f

void estimate_pose_from_particles()
{
    if (NUM_PARTICLES == 0)
        return;

    Vec2f avg_pos_mm = {0.0f, 0.0f};
    float avg_sin_rot = 0.0f;
    float avg_cos_rot = 0.0f;
    float total_weight = 0.0f;

    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        total_weight += particles[i].weight; // Should be 1.0 if normalized after resampling
        avg_pos_mm.x += particles[i].pos.x * particles[i].weight;
        avg_pos_mm.y += particles[i].pos.y * particles[i].weight;
        avg_sin_rot += sinf(particles[i].rot_rad) * particles[i].weight;
        avg_cos_rot += cosf(particles[i].rot_rad) * particles[i].weight;
    }

    if (total_weight > EPSILON)
    {
        avg_pos_mm.x /= total_weight;
        avg_pos_mm.y /= total_weight;
        avg_sin_rot /= total_weight;
        avg_cos_rot /= total_weight;
    }

    // Convert from particle frame (mm, rad from +X CCW) to POSE frame (m, deg [0,360) 0=+Y, 90=-X)
    float est_x_mm = avg_pos_mm.x;
    float est_y_mm = avg_pos_mm.y;

    float est_rot_rad_atan2 = atan2f(avg_sin_rot, avg_cos_rot); // Angle from +X, CCW
    // float est_rot_deg_atan2 = est_rot_rad_atan2 * 180.0f / PI; // [-180, 180]

    POSE.x = est_x_mm;
    POSE.y = est_y_mm;
    POSE.theta = normalize_angle_pi_pi(est_rot_rad_atan2);
}

bool checkTargetReached()
{
    if (currentMovement == IDLE || currentMovement == STOP_CMD)
        return true;

    float dx_mm = POSE.x - targetPose.x;
    float dy_mm = POSE.y - targetPose.y;
    float dist_sq_mm = dx_mm * dx_mm + dy_mm * dy_mm;

    float angle_diff_rad = normalize_angle_pi_pi(POSE.theta - targetPose.theta);
    angle_diff_rad = fabsf(angle_diff_rad); // Absolute difference

    float dist_thresh_mm, angle_thresh_rad;

    if (currentMovement == FWD)
    {
        dist_thresh_mm = 20.0f; // 20 mm

        // For FWD, primarily check distance. Angle is less critical once on path.
        return dist_sq_mm < (dist_thresh_mm * dist_thresh_mm);
    }
    else if (currentMovement == TURN_L || currentMovement == TURN_R)
    {
        angle_thresh_rad = 5 * (PI / 180); // 5 degrees
        // For turns, primary check is orientation.
        return angle_diff_rad < angle_thresh_rad;
    }
    return false;
}

#define TOF_FRONT_IDX ((int)TOF_Direction::FRONT)
#define TOF_LEFT_DIAG_IDX ((int)TOF_Direction::FRONT_LEFT_45)
#define TOF_RIGHT_DIAG_IDX ((int)TOF_Direction::FRONT_RIGHT_45)
#define TOF_SIDE_LEFT_IDX ((int)TOF_Direction::LEFT)
#define TOF_SIDE_RIGHT_IDX ((int)TOF_Direction::RIGHT)

bool front_wall_detected()
{
    if (MM_VALID[TOF_FRONT_IDX] && MM[TOF_FRONT_IDX] < (1.5 * CELL_SIZE_MM))
    {
        printf("Front wall detected at %dmm\n", MM[TOF_FRONT_IDX]);
        return true;
    }
    else
    {
        printf("No front wall detected.\n");
        return false;
    }
}

bool wall_left() {
    if (MM_VALID[TOF_SIDE_LEFT_IDX] && MM[TOF_SIDE_LEFT_IDX] < (1.5 * 1.41 * CELL_SIZE_MM / 2))
    {
        printf("Left wall detected at %dmm\n", MM[TOF_SIDE_LEFT_IDX]);
        return true;
    }
    else
    {
        printf("No left wall detected.\n");
        return false;
    }
}

bool wall_right() {
    if (MM_VALID[TOF_SIDE_RIGHT_IDX] && MM[TOF_SIDE_RIGHT_IDX] < (1.5 * 1.41 * CELL_SIZE_MM / 2))
    {
        printf("Right wall detected at %dmm\n", MM[TOF_SIDE_RIGHT_IDX]);
        return true;
    }
    else
    {
        printf("No right wall detected.\n");
        return false;
    }
}

int main() {
    printf("In main!\n");
    global_init();
    printf("Inited!\n");
    parse_maze_string(MAZE_ASCII_ART);
    printf("Parsed!\n");
    for (int r = 0; r < MAZE_SIZE; ++r)
    {
        for (int c = 0; c < MAZE_SIZE; ++c)
        {
            MOVE_MATRIX[r][c] = 255;
            if (MAZE_MATRIX[r][c].north)
            {
                MOVE_MATRIX[r][c] &= ~(1 << (int)Direction::TOP);
            }
            if (MAZE_MATRIX[r][c].south)
            {
                MOVE_MATRIX[r][c] &= ~(1 << (int)Direction::BOTTOM);
            }
            if (MAZE_MATRIX[r][c].west)
            {
                MOVE_MATRIX[r][c] &= ~(1 << (int)Direction::LEFT);
            }
            if (MAZE_MATRIX[r][c].east)
            {
                MOVE_MATRIX[r][c] &= ~(1 << (int)Direction::RIGHT);
            }
        }
    }

    print_maze();

    floodFill();

    printf("Finding fastest path...\n");
    Queue<Command> path = fastestPath();
    printf("Fastest path found!\n");
    Queue<Command> pathPrint = path;

    printf("Attempting to print path...\n");
    while (!pathPrint.empty()) {
        Command command = pathPrint.pop();
        printf("%s, %f\n", command.action.c_str(), command.value);
    }

    STATE = STATE_FAST_RUN;

    // while (true) {
    //     MotorL.setPWM(50);
    //     MotorR.setPWM(50);
    //     MotorL.update();
    //     MotorR.update();
    //     sleep_ms(1000);
    //     MotorL.setPWM(0);
    //     MotorR.setPWM(0);
    //     MotorL.update();
    //     MotorR.update();
    //     sleep_ms(5000);
    // }

    // while (true)
    // {
    //     if (stdio_usb_connected())
    //     {
    //         printf("---------- TESTING LEFT MOTOR -----------");
    //         motorTest(MotorL);

    //         printf("---------- TESTING RIGHT MOTOR -----------");
    //         motorTest(MotorR);
    //     }
    //     else
    //     {
    //         MotorR.setPWM(0);
    //         MotorL.setPWM(0);
    //     }
    // }

    //parse_maze_string(MAZE_ASCII_ART);
    
    print_maze();

    Queue commandQueue = {Command{"FWD", 3}, Command{"TRN", -PI/2}, Command{"FWD", 2}, Command{"STOP", 0}};
    // Queue commandQueue = {Command{"FWD", 1},  Command{"STOP", 0}};

    POSE.x = CELL_SIZE_MM * 0.5f; // Start at the center of the first cell
    POSE.y = CELL_SIZE_MM * 0.5f; // Start at the center of the first cell
    POSE.theta = PI / 2.0f;       // Facing "up" in the maze (0 degrees is +Y)
    printf("Initial pose: x=%.2fmm, y=%.2fmm, th=%.1fdeg\n", POSE.x, POSE.y, POSE.theta * 180.0f / (float)PI);

    resample_particles();

    uint64_t loop_time_prev_us = time_us_64();
    int loop_count = 0;

    setTarget(commandQueue.pop());

    while (true)
    {
        loop_count++;
        uint64_t loop_time_now_us = time_us_64();
        float dt = static_cast<float>(loop_time_now_us - loop_time_prev_us) / 1000000.0f;
        if (dt <= 0)
            dt = 0.02f; // Nominal dt if timer issue / first loop
        loop_time_prev_us = loop_time_now_us;

        global_read_tofs();
        global_read_imu();

        MotorL.update();
        MotorR.update();

        float dx_left_mm = MotorL.DELTA_POS;  // Delta mm for left wheel
        float dx_right_mm = MotorR.DELTA_POS; // Delta mm for right wheel

        float delta_s_body_mm = (dx_left_mm + dx_right_mm) / 2.0f; // Forward displacement of robot center
        float delta_theta_rad = (dx_right_mm - dx_left_mm) / ROBOT_WHEEL_BASE_MM;

        if (delta_s_body_mm > 0)
        {
            // printf("Robot is moving forward\n");
        }
        else if (delta_s_body_mm < 0)
        {
            // printf("Robot is moving backward\n");
        }
        else if (delta_theta_rad > EPSILON)
        {
            // printf("Robot is turning left\n");
        }
        else if (delta_theta_rad < -EPSILON)
        {
            // printf("Robot is turning right\n");
        }

        float rps = GYRO_Z * (float)PI / 180.0f; // Convert degrees per second to radians per second
        POSE.w = rps;                            // Update POSE.w with gyro reading

        // printf("Gyro Z: %.2f deg/s, %.2f rad/s\n", GYRO_Z, rps);

        float drot_rad_imu = rps * dt; // Change in rotation in radians

        // printf("Delta theta from motors: %.2f rad, Delta theta from IMU: %.2f rad\n", delta_theta_rad, drot_rad_imu);

        float odom_dx_world_mm = delta_s_body_mm * cosf(POSE.theta);
        float odom_dy_world_mm = delta_s_body_mm * sinf(POSE.theta);

        // printf("dx: %.2f mm, dy: %.2f mm\n", odom_dx_world_mm, odom_dy_world_mm);

        motion_update(odom_dx_world_mm, odom_dy_world_mm, drot_rad_imu);
        localize_particles(); // Update particle weights and resample

        // POSE.x += odom_dx_world_mm;
        // POSE.y += odom_dy_world_mm;
        estimate_pose_from_particles();

        CELL_X = (int)(POSE.x / CELL_SIZE_MM);
        CELL_Y = 15 - (int)(POSE.y / CELL_SIZE_MM);

        // printf("Estimated pose: x=%.2fmm, y=%.2fmm, th=%.1fdeg, cell=(%d,%d)\n", POSE.x, POSE.y, POSE.theta * 180.0f / (float)PI, CELL_X, CELL_Y);

        printf(">>> vizPARTICLES ");
        for (int i = 0; i < NUM_PARTICLES; i += 50)
        {
            printf("%.2f %.2f %.2f ", particles[i].pos.x, particles[i].pos.y, particles[i].rot_rad);
        }
        printf("\n");
        fflush(stdout);

        if (targetReached)
        {
            printf("Target reached or initial state. Planning next action.\n");
            printf("Current POSE before planning: x=%.2f, y=%.2f, th=%.1f\n", POSE.x, POSE.y, POSE.theta * 180.0f / (float)PI);

            VContr.reset(); // Reset PID integrators for new maneuver
            WContr.reset();

            // Command next_cmd = decide_next_action_tof_based();
            // Command next_cmd = commandQueue.pop();
            // printf("Next command: %s, value: %.1f\n", next_cmd.action.c_str(), next_cmd.value);

            // setTarget(next_cmd); // Updates global TARGET_POSE, prevTARGET_POSE, CURRENT_MOVEMENT, targetReached=false

            // printf("New target set: x=%.2fm, y=%.2fm, th=%.1fdeg. Movement: %d. Target V:%.2f W:%.2f\n",
            //        TARGET_POSE.x, TARGET_POSE.y, TARGET_POSE.theta, static_cast<int>(CURRENT_MOVEMENT), TARGET_POSE.v, TARGET_POSE.w);
        }

        float rpm_L = MotorL.RPM;
        float rpm_R = MotorR.RPM;
        float v_wheel_L_mmps = rpm_L * (1.0f / 60.0f) * (2.0f * PI * WHEEL_RADIUS_MM);
        float v_wheel_R_mmps = rpm_R * (1.0f / 60.0f) * (2.0f * PI * WHEEL_RADIUS_MM);

        POSE.v = (v_wheel_L_mmps + v_wheel_R_mmps) / 2.0f; // m/s
        // POSE.w = (rpm_R - rpm_L) / (WHEEL_BASE_M) * (2.0f * PI * WHEEL_RADIUS_M) / 60.0f; // rad/s (R>L means CCW for math angle, check convention)
        //                                                                                     // If using "setTarget" theta, R>L is a left turn (positive W)

        // Get desired angular velocity from Stanley controller
        float w_ref_stanley_radps = StanContr.output();

        float v_effort = VContr.output(targetPose.v, POSE.v); // targetPose.v in mm/s
        float w_effort = WContr.output(targetPose.w, POSE.w); // targetPose.w (from Stanley) in rad/s

        float duty_L_percent = v_effort - w_effort;
        float duty_R_percent = v_effort + w_effort;

        duty_L_percent = std::max(-100.0f, std::min(100.0f, duty_L_percent));
        duty_R_percent = std::max(-100.0f, std::min(100.0f, duty_R_percent));

        MotorL.setPWM(duty_L_percent);
        MotorR.setPWM(duty_R_percent);

        // 6. UPDATE STATE (Check if target is reached)
        if (!targetReached)
        {
            targetReached = checkTargetReached();
            if (targetReached)
            {
                Command next_cmd = commandQueue.pop();
                setTarget(next_cmd);
                printf("Next command: %s, value: %.1f\n", next_cmd.action.c_str(), next_cmd.value);

                // currentMovement = IDLE; // Optional: transition to IDLE
            }
        }

        // --- Logging ---
        if (loop_count % 5 == 0)
        {
            printf("Target Pose: x=%.2fmm, y=%.2fmm, th=%.1fdeg, v=%.2fmm/s, w=%.2frad/s\n",
                   targetPose.x, targetPose.y, targetPose.theta * 180.0f / (float)PI,
                   targetPose.v, targetPose.w);
            printf("Target Pose / CELL_WIDTH: (%f, %f)\n", (float)(targetPose.x / CELL_SIZE_MM), 15 - (float)(targetPose.y / CELL_SIZE_MM));

        }
        sleep_ms(1);
    }
    
    if (STATE = STATE_FAST_RUN) {
        // Init fastest path 
        floodFill();
        commandQueue = fastestPath();

        setTarget(commandQueue.pop());
        while (true) {


        }
    }

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
