#pragma once

#include "API.h"

#include <random>

struct Particle
{
    Vec2f pos;     // mm
    float rot_rad; // radians, [-PI, PI), 0 along +X, PI/2 along +Y
    float weight;
};

struct RayMarchResult
{
    Vec2f point;
    bool hit_something; // True if wall or grid boundary hit, false if max range exceeded or invalid ray
};

inline RayMarchResult fast_ray_march_cpp(
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

inline Vec2f rotate_vector_cpp(const Vec2f &v, float angle_rad)
{
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);
    return Vec2f(
        v.x * cos_a - v.y * sin_a,
        v.x * sin_a + v.y * cos_a);
}

constexpr float SENSOR_ORIENTATIONS_RAD[NUM_TOF_SENSORS] = {
    0.0f,
    (float)PI / 2.0f,
    -(float)PI / 2.0f,
    (float)PI / 4.0f,
    -(float)PI / 4.0f};

inline Vec2f get_tof_location_cpp(const Vec2f &robot_pos_world, float robot_rot_rad, int sensor_index)
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

inline std::random_device random_device;
inline std::mt19937 rand_gen(random_device());

inline void update_tof_sensor_data_ray_marching_cpp(
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

inline void print_maze()
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
inline const char *MAZE_ASCII_ART =
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

inline void parse_maze_string(const char *maze_str_input)
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
                MAZE_MATRIX[cell_r][cell_c].north = 1;
            }

            // South Wall: on text line `(cell_r * 2) + 2`, at character `cell_c * 4 + 2` within that line.
            int south_wall_text_line_idx = cell_r * 2 + 2;
            int south_wall_char_idx_in_line = cell_c * 4 + 2;
            char south_char = current_pos[south_wall_text_line_idx * string_stride + south_wall_char_idx_in_line];
            if (south_char == '-')
            {
                MAZE_MATRIX[cell_r][cell_c].south = 1;
            }

            // West Wall: on text line `(cell_r * 2) + 1`, at character `cell_c * 4` within that line.
            int west_wall_text_line_idx = cell_r * 2 + 1;
            int west_wall_char_idx_in_line = cell_c * 4;
            char west_char = current_pos[west_wall_text_line_idx * string_stride + west_wall_char_idx_in_line];
            if (west_char == '|')
            {
                MAZE_MATRIX[cell_r][cell_c].west = 1;
            }

            // East Wall: on text line `(cell_r * 2) + 1`, at character `(cell_c * 4) + 4` within that line.
            int east_wall_text_line_idx = cell_r * 2 + 1;
            int east_wall_char_idx_in_line = cell_c * 4 + 4;
            char east_char = current_pos[east_wall_text_line_idx * string_stride + east_wall_char_idx_in_line];
            if (east_char == '|')
            {
                MAZE_MATRIX[cell_r][cell_c].east = 1;
            }

            // Visited is not set by this parser, defaults to 0.
            MAZE_MATRIX[cell_r][cell_c].visited = 0;
        }
    }
}