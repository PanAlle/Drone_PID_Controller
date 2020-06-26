#include "drone_simulation.h"

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

#define DRONE_MAX_LANDING_SPEED -0.5f  // [m/s]

static FILE *log_file = NULL;
static float user_data_to_log;
static struct vehicle {
    float height;       // [m]
    float speed;        // [m/s]
    float acceleration; // [m/s^2]
    bool crashed;
} drone;

float sim_advance_time(void)
{
    static float time = 0.0f; // [s]

    assert(log_file != NULL); // called before sim_start
    fprintf(log_file, "%f, %f, %f, %f, %f\n",
            time, drone.height, drone.speed, drone.acceleration, user_data_to_log);

    if(drone.crashed == false) {
        float previous_height = drone.height;
        float acceleration = drone.acceleration +
                             EARTH_ACCELERATION;
        // height difference due to prior velocity
        drone.height += drone.speed * TIME_STEP;
        // height difference due to changing acceleration
        drone.height += 0.5f * acceleration * TIME_STEP * TIME_STEP;
        // change in speed due to change in acceleration
        drone.speed += acceleration * TIME_STEP;

        if (drone.height <= 0.0f) {
            if(previous_height > 0.0f) {
                if(drone.speed < DRONE_MAX_LANDING_SPEED) {
                    printf("Drone crashed with speed %f!\n", -drone.speed);
                    drone.crashed = true;
                } else {
                    printf("Drone landed!\n");
                }
            }
            drone.speed = 0.0f;
            drone.height = 0.0f;
        }
    }

    time += TIME_STEP;
    return time;
}

float sim_get_height(void)
{
    return drone.height;
}

void sim_set_acceleration(float acceleration)
{
    assert(acceleration >= MIN_ACCELERATION);
    assert(acceleration <= MAX_ACCELERATION);
    drone.acceleration = acceleration * -EARTH_ACCELERATION - EARTH_ACCELERATION;
}

void sim_set_user_data(float user_data)
{
    user_data_to_log = user_data;
}

void sim_start(const char *filename)
{
    assert(filename != NULL); // no path given
    assert(filename[0] != '\0'); // empty path given
    log_file = fopen(filename, "w");
    assert(log_file != NULL); // file creation failed
    fprintf(log_file, "time, height, speed, acceleration, user_data\n");
    sim_advance_time();
}

void sim_end(void)
{
    assert(log_file != NULL); // called before sim_start
    fclose(log_file);
}
