#include "drone_simulation.h"

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

typedef struct Error {
    float prp_error;
    float der_error;
    float int_error;
    float correction_val;

} error;

typedef struct PID {
    float P;
    float I;
    float D;
} pid;

int main() {
    //Initialize error structure and pid parameters
    pid pid = {10, 0, 2};
    error err = {0, 0, 0};
    float pre_err = 0;
    //Start the simulation
    sim_start("C:\\Users\\utente\\CLionProjects\\Drone_PID_Controller\\Drone_PID_Controller.csv");
    float sim_time = 0;
    printf("Press 1 for simulating a step function, press 2 to simulate a trajectory: \n");
    char selection = '0';
    scanf("%c", &selection);

    //Simulate for 30 seconds
    while (sim_time <= 30) {
        //Select the different simulation with 1 or 2.
        //1 - Step function
        //2 - Trajectory following

        //Step function
        if (selection == '1') {
            sim_set_user_data(1);
            err.prp_error = 1 - sim_get_height();
        }
        // Trajectory following
        if (selection == '2') {
            sim_set_user_data(sim_get_example_trajectory(sim_time));
            err.prp_error = sim_get_example_trajectory(sim_time) - sim_get_height();
        }

        //Compute error for PID controller
        err.der_error = (err.prp_error - pre_err) / TIME_STEP;
        err.int_error += (err.prp_error * TIME_STEP);
        //Save old error for next iteration
        pre_err = err.prp_error;
        //Define new acceleration based on the PID parameters
        err.correction_val = pid.P * err.prp_error + pid.D * err.der_error + pid.I * err.int_error;
        //Limit the max/min correction value of the acceleration
        if (err.correction_val >= MAX_ACCELERATION) err.correction_val = MAX_ACCELERATION;
        else if (err.correction_val <= MIN_ACCELERATION) err.correction_val = MIN_ACCELERATION;
        //Set the acceleration and iterate the simulation
        sim_set_acceleration(err.correction_val);
        sim_time = sim_advance_time();
    }
    //End simulation
    sim_end();
    return 0;
}
