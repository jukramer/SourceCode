#include "common.h"
#include "API.h"
#include "particles.h"

#include <string>
#include <vector>
#include <random>
#include <time.h>
#include <string.h>
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
#define V_MAX 200.0f                        // mm/s (Reduced V_MAX for more stable testing initially)
#define TURN_RADIUS (CELL_WIDTH / 2)        // mm
#define W_MAX_NOMINAL (V_MAX / TURN_RADIUS) // Nominal angular velocity for turns (rad/s)

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
int TARGET_CELL_X = 0;
int TARGET_CELL_Y = 0;

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

VController VContr(KP_V, KI_V, 0.0f /*KD_V not used*/);
WController WContr(KP_W, KI_W, 0.0f /*KD_W not used*/);

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

Direction theta_to_direction(float theta)
{
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

float direction_to_theta(Direction dir)
{
    switch (dir)
    {
    case Direction::RIGHT:
        return 0.0f; // +X
    case Direction::TOP:
        return PI / 2.0f; // +Y
    case Direction::LEFT:
        return PI; // -X
    case Direction::BOTTOM:
        return -PI / 2.0f; // -Y
    default:
        return 0.0f; // Default case, should not happen
    }
}

void setTargetSmooth(Command command)
{
    prevTargetPose = targetPose; // The target we just arrived at (or current POSE if first command)
    targetReached = false;

    if (command.action == "FWD")
    {
        currentMovement = FWD;

        Direction dir = theta_to_direction(POSE.theta);

        printf("FWD command: %s, value: %.2f, dir: %d\n", command.action.c_str(), command.value, dir);
        int dir_index = static_cast<int>(dir);

        int cells_forward = static_cast<int>(command.value);

        TARGET_CELL_X = CELL_X + OFFSET_LOCATIONS[dir_index].x * cells_forward;
        TARGET_CELL_Y = CELL_Y + OFFSET_LOCATIONS[dir_index].y * cells_forward;

        targetPose.theta = direction_to_theta(dir);
        targetPose.v = V_MAX;
        targetPose.w = 0.0f;

        // Calculate wall-aligned stop point
        targetPose.x = TARGET_CELL_X * CELL_SIZE_MM + (dir == Direction::LEFT ? CELL_SIZE_MM : 0) + CELL_SIZE_MM / 2.0f;
        targetPose.y = (15 - TARGET_CELL_Y) * CELL_SIZE_MM + (dir == Direction::BOTTOM ? CELL_SIZE_MM : 0) + CELL_SIZE_MM / 2.0f;

        targetPose.x -= (CELL_SIZE_MM / 2.0f + LOCAL_TOF_BASE_OFFSET_X_MM) * OFFSET_LOCATIONS[dir_index].x;
        targetPose.y += (CELL_SIZE_MM / 2.0f + LOCAL_TOF_BASE_OFFSET_X_MM) * OFFSET_LOCATIONS[dir_index].y;
    }
    else if (command.action == "TRN")
    {
        float turn_angle_rad = command.value; // radians, positive for left (CCW)

        // Center of current cell, nudged by half a cell in turn direction
        Direction dir = theta_to_direction(POSE.theta);
        int dir_index = static_cast<int>(dir);

        Direction dir_after;
        if (turn_angle_rad > 0)
        {
            currentMovement = TURN_L;
            dir_after = ROTATE_LEFT[dir_index];
        }
        else
        {
            currentMovement = TURN_R;
            dir_after = ROTATE_RIGHT[dir_index];
        }
        currentMovement = (turn_angle_rad > 0) ? TURN_L : TURN_R;

        targetPose.theta = normalize_angle_pi_pi(direction_to_theta(dir) + turn_angle_rad);

        targetPose.v = V_MAX * 0.8f;
        targetPose.w = (turn_angle_rad > 0 ? W_MAX_NOMINAL : -W_MAX_NOMINAL) * 0.8f;

        TARGET_CELL_X = CELL_X + OFFSET_LOCATIONS[static_cast<int>(dir_after)].x;
        TARGET_CELL_Y = CELL_Y + OFFSET_LOCATIONS[static_cast<int>(dir_after)].y;

        targetPose.x = TARGET_CELL_X * CELL_SIZE_MM + (dir == Direction::LEFT ? CELL_SIZE_MM : 0);
        targetPose.y = (15 - TARGET_CELL_Y) * CELL_SIZE_MM + (dir == Direction::BOTTOM ? CELL_SIZE_MM : 0);

        if (dir_after == Direction::RIGHT)
        {
            // targetPose.x += LOCAL_TOF_BASE_OFFSET_X_MM;
            targetPose.y += CELL_SIZE_MM / 2.0f; // Center in cell
        }
        if (dir_after == Direction::LEFT)
        {
            // targetPose.x -= LOCAL_TOF_BASE_OFFSET_X_MM;
            targetPose.y += CELL_SIZE_MM / 2.0f; // Center in cell
        }
        if (dir_after == Direction::TOP)
        {
            targetPose.x += CELL_SIZE_MM / 2.0f; // Center in cell
            // targetPose.y += LOCAL_TOF_BASE_OFFSET_X_MM / 2;
        }
        if (dir_after == Direction::BOTTOM)
        {
            targetPose.x += CELL_SIZE_MM / 2.0f; // Center in cell
            targetPose.y += CELL_SIZE_MM / 2.0f; // Adjust for local sensor offset
            // targetPose.y -= LOCAL_TOF_BASE_OFFSET_X_MM;
        }
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

    printf("Target Set when CELL_X: %d, CELL_Y: %d\n", CELL_X, CELL_Y);
    printf("> TARGET_CELL_X: %d, TARGET_CELL_Y: %d\n", TARGET_CELL_X, TARGET_CELL_Y);
    printf("> Target Pose: x: %.2f, y: %.2f, theta: %.2f, v: %.2f, w: %.2f\n",
           targetPose.x, targetPose.y, targetPose.theta, targetPose.v, targetPose.w);
}

void setTarget(Command command)
{
    prevTargetPose = targetPose; // The target we just arrived at (or current POSE if first command)
    targetReached = false;

    if (command.action == "FWD")
    {
        currentMovement = FWD;

        Direction dir = theta_to_direction(POSE.theta);

        printf("FWD command: %s, value: %.2f, dir: %d\n", command.action.c_str(), command.value, dir);
        int dir_index = static_cast<int>(dir);

        int cells_forward = static_cast<int>(command.value);

        TARGET_CELL_X = CELL_X + OFFSET_LOCATIONS[dir_index].x * cells_forward;
        TARGET_CELL_Y = CELL_Y + OFFSET_LOCATIONS[dir_index].y * cells_forward;

        targetPose.theta = direction_to_theta(dir);
        targetPose.v = V_MAX;
        targetPose.w = 0;

        targetPose.x = TARGET_CELL_X * CELL_SIZE_MM + CELL_SIZE_MM / 2.0f;
        targetPose.y = (15 - TARGET_CELL_Y) * CELL_SIZE_MM + CELL_SIZE_MM / 2.0f;

        // targetPose.x += (LOCAL_TOF_BASE_OFFSET_X_MM / 3) * OFFSET_LOCATIONS[dir_index].x;
        // targetPose.y -= (LOCAL_TOF_BASE_OFFSET_X_MM / 3) * OFFSET_LOCATIONS[dir_index].y;
    }
    else if (command.action == "TRN")
    {
        float turn_angle_rad = command.value; // radians, positive for left (CCW)

        // Center of current cell, nudged by half a cell in turn direction
        Direction dir = theta_to_direction(POSE.theta);
        int dir_index = static_cast<int>(dir);

        Direction dir_after;
        if (turn_angle_rad > 0)
        {
            currentMovement = TURN_L;
            dir_after = ROTATE_LEFT[dir_index];
        }
        else
        {
            currentMovement = TURN_R;
            dir_after = ROTATE_RIGHT[dir_index];
        }
        currentMovement = (turn_angle_rad > 0) ? TURN_L : TURN_R;

        targetPose.theta = normalize_angle_pi_pi(direction_to_theta(dir) + turn_angle_rad);

        targetPose.v = 0; // V_MAX * 0.8f;
        targetPose.w = (turn_angle_rad > 0 ? W_MAX_NOMINAL : -W_MAX_NOMINAL);

        TARGET_CELL_X = CELL_X; // + OFFSET_LOCATIONS[static_cast<int>(dir_after)].x;
        TARGET_CELL_Y = CELL_Y; // + OFFSET_LOCATIONS[static_cast<int>(dir_after)].y;

        targetPose.x = POSE.x; // TARGET_CELL_X * CELL_SIZE_MM + (dir == Direction::LEFT ? CELL_SIZE_MM : 0);
        targetPose.y = POSE.y; // (15 - TARGET_CELL_Y) * CELL_SIZE_MM + (dir == Direction::BOTTOM ? CELL_SIZE_MM : 0);

        /*if (dir_after == Direction::RIGHT) {
            //targetPose.x += LOCAL_TOF_BASE_OFFSET_X_MM;
            targetPose.y += CELL_SIZE_MM / 2.0f; // Center in cell
        }
        if (dir_after == Direction::LEFT) {
            //targetPose.x -= LOCAL_TOF_BASE_OFFSET_X_MM;
            targetPose.y += CELL_SIZE_MM / 2.0f; // Center in cell
        }
        if (dir_after == Direction::TOP) {
            targetPose.x += CELL_SIZE_MM / 2.0f; // Center in cell
            //targetPose.y += LOCAL_TOF_BASE_OFFSET_X_MM / 2;
        }
        if (dir_after == Direction::BOTTOM) {
            targetPose.x += CELL_SIZE_MM / 2.0f; // Center in cell
            targetPose.y += CELL_SIZE_MM / 2.0f; // Adjust for local sensor offset
            //targetPose.y -= LOCAL_TOF_BASE_OFFSET_X_MM;
        }*/
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

    printf("Target Set when CELL_X: %d, CELL_Y: %d\n", CELL_X, CELL_Y);
    printf("> TARGET_CELL_X: %d, TARGET_CELL_Y: %d\n", TARGET_CELL_X, TARGET_CELL_Y);
    printf("> Target Pose: x: %.2f, y: %.2f, theta: %.2f, v: %.2f, w: %.2f\n",
           targetPose.x, targetPose.y, targetPose.theta, targetPose.v, targetPose.w);
}

#define NUM_PARTICLES 100

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

Vec2f sample_around_pose(float radius = 100.0f)
{
    radius = rand_uniform(0.0f, radius);
    float angle = random_angle();
    float x = POSE.x + radius * cosf(angle);
    float y = POSE.y + radius * sinf(angle);
    return Vec2f(x, y);
}

void resample_particles(float radius = 100.0f)
{
    for (int i = 0; i < NUM_PARTICLES; ++i)
    {
        particles[i].pos = sample_around_pose(radius);
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

    if (weight_sum > 1e-6f && valid_sensor_count > 0)
    {
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

            // These noise values are critical for diversity after resampling.
            new_particles[m].pos.x += rand_gauss(0.0f, 0.5f);    // mm (tune this)
            new_particles[m].pos.y += rand_gauss(0.0f, 0.5f);    // mm (tune this)
            new_particles[m].rot_rad += rand_gauss(0.0f, 0.02f); // radians (tune this)
        }
        std::swap(particles, new_particles);
    }
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

        float noise_x = rand_gauss(0.0f, 0.1f);     // mm
        float noise_y = rand_gauss(0.0f, 0.1f);     // mm
        float noise_rot = rand_gauss(0.0f, 0.005f); // radians

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
        Direction dir = theta_to_direction(POSE.theta);
        if (dir == Direction::RIGHT || dir == Direction::LEFT)
        {
            return abs(dx_mm) < 5.0f;
        }
        else
        {
            return abs(dy_mm) < 5.0f;
        }
        //dist_thresh_mm = 10.0f; // 20 mm

        // For FWD, primarily check distance. Angle is less critical once on path.
        //return dist_sq_mm < (dist_thresh_mm * dist_thresh_mm);
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

bool wall_front()
{
    if (MM_VALID[TOF_FRONT_IDX] && MM[TOF_FRONT_IDX] < (1.2 * CELL_SIZE_MM))
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

bool wall_left()
{
    if (MM_VALID[TOF_LEFT_DIAG_IDX] && MM[TOF_LEFT_DIAG_IDX] < (1.4 * 1.41 * CELL_SIZE_MM / 2))
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

bool wall_right()
{
    if (MM_VALID[TOF_RIGHT_DIAG_IDX] && MM[TOF_RIGHT_DIAG_IDX] < (1.4 * 1.41 * CELL_SIZE_MM / 2))
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

void scanFrontWall()
{
    if (MM_VALID[TOF_FRONT_IDX])
    {
        if (direction_to_theta(theta_to_direction(POSE.theta)) - POSE.theta > 0.1f)
        {
            printf("Front wall scan skipped due to pose direction mismatch.\n");
            return;
        }

        float front_distance_cells = round((MM[TOF_FRONT_IDX] - LOCAL_TOF_BASE_OFFSET_X_MM) / CELL_SIZE_MM);
        Direction dir = theta_to_direction(POSE.theta);
        int cell_x = CELL_X + OFFSET_LOCATIONS[dir].x * front_distance_cells;
        int cell_y = CELL_Y + OFFSET_LOCATIONS[dir].y * front_distance_cells;

        // printf("Front wall detected at %.1d mm, cell (%d, %d)\n", MM[TOF_FRONT_IDX], cell_x, cell_y);
        // printf("Current pose: x=%.2fmm, y=%.2fmm, theta=%.1fdeg\n", POSE.x, POSE.y, POSE.theta * 180.0f / (float)PI);
        // printf("Current cell: (%d, %d)\n", CELL_X, CELL_Y);
        setWall(cell_x, cell_y, dir);
    }
    else
    {
        printf("Front sensor not valid.\n");
    }
}

void scanWalls()
{
    bool front_wall = wall_front();
    bool left_wall = wall_left();
    bool right_wall = wall_right();

    Direction dir = theta_to_direction(POSE.theta);
    int cell_x = CELL_X + OFFSET_LOCATIONS[dir].x;
    int cell_y = CELL_Y + OFFSET_LOCATIONS[dir].y;

    if (front_wall)
    {
        setWall(cell_x, cell_y, dir);
    }
    if (left_wall)
    {
        setWall(cell_x, cell_y, ROTATE_LEFT[dir]);
    }
    if (right_wall)
    {
        setWall(cell_x, cell_y, ROTATE_RIGHT[dir]);
    }
}

bool left_wall_seen()
{
    if (MM_VALID[TOF_SIDE_LEFT_IDX] && MM[TOF_SIDE_LEFT_IDX] < (1.5 * CELL_SIZE_MM))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool right_wall_seen()
{
    if (MM_VALID[TOF_SIDE_RIGHT_IDX] && MM[TOF_SIDE_RIGHT_IDX] < (1.5 * CELL_SIZE_MM))
    {
        return true;
    }
    else
    {
        return false;
    }
}

float get_cross_track_error()
{
    float feedback = 0.0f;

    bool left_wall = left_wall_seen();
    bool right_wall = right_wall_seen();

    float left_error = CELL_SIZE_MM / 2 - MM[TOF_SIDE_LEFT_IDX];
    float right_error = CELL_SIZE_MM / 2 - MM[TOF_SIDE_RIGHT_IDX];

    float error = 0.0f;
    if (left_wall && right_wall)
    {
        error = left_error - right_error;
    }
    else if (left_wall)
    {
        error = 2 * left_error;
    }
    else if (right_wall)
    {
        error = -2 * right_error;
    }

    return error;
}

#define STEERING_KP 0.1f
#define STEERING_KD 0.01f
#define STEERING_ADJUST_LIMIT 0.1f // Max adjustment in radians

float last_cross_track_error = 0.0f;

float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float calculate_steering_adjustment(float cross_track_error, float dt) {
    // always calculate the adjustment for testing. It may not get used.
    float pTerm = STEERING_KP * cross_track_error;
    float dTerm = STEERING_KD * (cross_track_error - last_cross_track_error);
    float adjustment = pTerm + dTerm * dt;

    adjustment = constrain(adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
    last_cross_track_error = cross_track_error;
    return adjustment;
}

bool atCellBoundary()
{
    Direction dir = theta_to_direction(POSE.theta);

    float x = fmodf(POSE.x + OFFSET_LOCATIONS[dir].x * LOCAL_TOF_BASE_OFFSET_X_MM, CELL_SIZE_MM);
    float y = fmodf(POSE.y - OFFSET_LOCATIONS[dir].y * LOCAL_TOF_BASE_OFFSET_X_MM, CELL_SIZE_MM);
    // printf("At cell boundary check: x=%.2fmm, y=%.2fmm\n", x, y);

    bool at_boundary = (fabsf(x - CELL_SIZE_MM) < 2) || (fabsf(y - CELL_SIZE_MM) < 2);
    if (at_boundary)
    {
        printf("At cell boundary: x=%.2fmm, y=%.2fmm\n", POSE.x, POSE.y);
    }
    return at_boundary;
}

int main()
{
    global_init();

   /* while (true) {
        motorTest(MotorL);
        motorTest(MotorR);

        sleep_ms(1000);
    }*/

    // parse_maze_string(MAZE_ASCII_ART);

    // IMPORTANT: Flood fill doesnt work if no outer walls
    memset(MAZE_MATRIX, 0, sizeof(MAZE_MATRIX));
    memset(MOVE_MATRIX, 255, sizeof(MOVE_MATRIX));

    for (int r = 0; r < MAZE_SIZE; ++r)
    {
        for (int c = 0; c < MAZE_SIZE; ++c)
        {
            if (r == 0)
            {
                setWall(c, r, Direction::TOP); // Top row is always a wall
            }
            if (r == MAZE_SIZE - 1)
            {
                setWall(c, r, Direction::BOTTOM); // Bottom row is always a wall
            }
            if (c == 0)
            {
                setWall(c, r, Direction::LEFT);
            }
            if (c == MAZE_SIZE - 1)
            {
                setWall(c, r, Direction::RIGHT); // Right column is always a wall
            }
        }
    }

    printf("Finding fastest path...\n");
    Queue<Command> path = fastestPath();
    printf("Fastest path found!\n");
    Queue<Command> pathPrint = path;

    printf("Attempting to print path...\n");
    while (!pathPrint.empty())
    {
        Command command = pathPrint.pop();
        printf("%s, %f\n", command.action.c_str(), command.value);
    }

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

    STATE = STATE_MAP_EXPLORE;

    // Queue commandQueue = {Command{"FWD", 3}, Command{"TRN", -PI/2}, Command{"FWD", 2}, Command{"STOP", 0}};
    Queue commandQueue = {Command{"FWD", 1}};

    POSE.x = 80.0;          // Start at the center of the first cell
    POSE.y = 90.0;          // Start at the center of the first cell
    POSE.theta = PI / 2.0f; // Facing "up" in the maze (0 degrees is +Y)

    CELL_X = (int)(POSE.x / CELL_SIZE_MM);
    CELL_Y = 15 - (int)(POSE.y / CELL_SIZE_MM);

    printf("Initial pose: x=%.2fmm, y=%.2fmm, th=%.1fdeg\n", POSE.x, POSE.y, POSE.theta * 180.0f / (float)PI);

    resample_particles(10.0f);

    uint64_t loop_time_prev_us = time_us_64();
    int loop_count = 0;

    setTarget(commandQueue.pop());

    floodFill();

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

        {
            // Time how long it takes to update the pose
            uint64_t start_time = time_us_64();

            motion_update(odom_dx_world_mm, odom_dy_world_mm, drot_rad_imu);
            localize_particles(); // Update particle weights and resample

            if (false) // (currentMovement == FWD)
            {
                // Update pose based on particles
                estimate_pose_from_particles();
            }
            else
            {
                POSE.x += odom_dx_world_mm;
                POSE.y += odom_dy_world_mm;
                POSE.theta += drot_rad_imu;
                // resample_particles(1.0f); // Resample particles around last pose
            }

            uint64_t end_time = time_us_64();
            uint64_t elapsed_time = end_time - start_time;
            // printf("Particles took %llu us\n", elapsed_time);
        }

        CELL_X = (int)(POSE.x / CELL_SIZE_MM);
        CELL_Y = 15 - (int)(POSE.y / CELL_SIZE_MM);

        if (atCellBoundary() && currentMovement == FWD)
        {
            scanWalls();
            scanFrontWall();
        }

        if (FLOOD_DIRTY)
        {
            floodFill();
            printMaze();
            FLOOD_DIRTY = false;
        }

        // printf("Estimated pose: x=%.2fmm, y=%.2fmm, th=%.1fdeg, cell=(%d,%d)\n", POSE.x, POSE.y, POSE.theta * 180.0f / (float)PI, CELL_X, CELL_Y);

        printf(">>> vizPARTICLES ");
        for (int i = 0; i < NUM_PARTICLES; i += 50)
        {
            printf("%.2f %.2f %.2f ", particles[i].pos.x, particles[i].pos.y, particles[i].rot_rad);
        }
        printf("\n");
        fflush(stdout);

        printf(">>> vizPOSE %.2f %.2f %.2f\n", POSE.x, POSE.y, POSE.theta);
        fflush(stdout);

        printf(">>> vizTARGET %.2f %.2f %.2f\n", targetPose.x, targetPose.y, targetPose.theta);
        fflush(stdout);

        float rpm_L = MotorL.RPM;
        float rpm_R = MotorR.RPM;
        float v_wheel_L_mmps = rpm_L * (1.0f / 60.0f) * (2.0f * PI * WHEEL_RADIUS_MM);
        float v_wheel_R_mmps = rpm_R * (1.0f / 60.0f) * (2.0f * PI * WHEEL_RADIUS_MM);

        POSE.v = (v_wheel_L_mmps + v_wheel_R_mmps) / 2.0f; // m/s
        // POSE.w = (rpm_R - rpm_L) / (WHEEL_BASE_M) * (2.0f * PI * WHEEL_RADIUS_M) / 60.0f; // rad/s (R>L means CCW for math angle, check convention)
        //                                                                                     // If using "setTarget" theta, R>L is a left turn (positive W)

        float cross_track_error = get_cross_track_error();
        float steering_feedback = calculate_steering_adjustment(cross_track_error, dt);

        printf("Steering feedback: %.2f\n", steering_feedback);
        POSE.w += steering_feedback;

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
                VContr.reset(); // Reset PID integrators for new maneuver
                WContr.reset();

                Direction minFloodFillDir;
                int minFloodFill = 255;
                int x1, y1;

                uint8_t mask = MOVE_MATRIX[CELL_Y][CELL_X];
                for (int i = 0; i < 4; ++i)
                {
                    if (!(mask & (1 << i)))
                        continue;

                    int nx = CELL_X + OFFSET_LOCATIONS[i].x;
                    int ny = CELL_Y + OFFSET_LOCATIONS[i].y;

                    if (FLOOD_MATRIX[ny][nx] <= minFloodFill)
                    {
                        minFloodFill = FLOOD_MATRIX[ny][nx];
                        minFloodFillDir = (Direction)i;
                        x1 = nx;
                        y1 = ny;
                    }
                }

                Command nextCommand;

                Direction currentDir = theta_to_direction(POSE.theta);
                if (currentDir == minFloodFillDir)
                {
                    nextCommand = Command{"FWD", 1};
                }
                else if (minFloodFillDir - currentDir == -1 || minFloodFillDir - currentDir == 3)
                {
                    nextCommand = Command {"TRN", PI/2.0};
                }
                else if (minFloodFillDir - currentDir == 1 || minFloodFillDir - currentDir == -3)
                {
                    nextCommand = Command {"TRN", -PI/2.0};
                }
                setTarget(nextCommand);

                scanFrontWall();

                printf("---------------------------------------------Target reached! New target set.\n");
                printf("Next command: %s, value: %.1f\n", nextCommand.action.c_str(), nextCommand.value);

                // currentMovement = IDLE; // Optional: transition to IDLE
            }
        }

        // --- Logging ---
        if (loop_count % 5 == 0)
        {
            // printf("CELL_X: %d, CELL_Y: %d\n", CELL_X, CELL_Y);
            // printf("TARGET_CELL_X: %d, TARGET_CELL_Y: %d\n", TARGET_CELL_X, TARGET_CELL_Y);

            /*printf("T:%.2fs|POSE(mm,rad,v,w):%.0f,%.0f,%.2f|%.0f,%.2f|TGT:%.0f,%.0f,%.2f|%.0f,%.2f|R:%d|Mv:%d|StW:%.2f|Duty:%.0f,%.0f\n",
              loop_time_now_us/1e6f, POSE.x, POSE.y, POSE.theta, POSE.v, POSE.w,
              targetPose.x, targetPose.y, targetPose.theta, targetPose.v, targetPose.w,
              targetReached, static_cast<int>(currentMovement), w_ref_stanley_radps, duty_L_percent, duty_R_percent);*/
        }
    }

    if (STATE = STATE_FAST_RUN)
    {
        // Init fastest path
        floodFill();
        commandQueue = fastestPath();

        setTarget(commandQueue.pop());
        while (true)
        {
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
