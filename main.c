#include "drone_simulation.h"

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

typedef struct Error{
    float prp_error;
    float der_error;
    float int_error;
    float correction_val;

} error;

typedef struct PID{
   float P;
   float I;
   float D;
}pid;

int main() {
    //Initialize error structure and pid parameters
    pid pid = {2,0.5,0.2 };
    error err = { 0, 0, 0};
    float pre_error = 0;
    //Start the simulation
    sim_start("C:\\Users\\utente\\CLionProjects\\Drone_PID_Controller\\Drone_PID_Controller.csv");
    float sim_time = 0;
    //Simulate for 30 seconds
    while (sim_time <= 30){

        // Trajectory following case
        //sim_set_user_data(sim_get_example_trajectory(sim_time));
        //err.prp_error = sim_get_example_trajectory(sim_time) - sim_get_height();

        //Step function
        sim_set_user_data(1);
        err.prp_error = 1 - sim_get_height();
        err.der_error = (err.prp_error - pre_error)/TIME_STEP;
        err.int_error += err.prp_error*TIME_STEP;

        pre_error = err.prp_error;
        //Define new accelleration based on the pid parameters
        err.correction_val = pid.P  * err.prp_error + pid.D * err.der_error + pid.I * err.int_error;
        printf("Acc correction%f\n", err.correction_val);
        sim_set_acceleration(err.correction_val);
        sim_time = sim_advance_time();
    }
    //End simulation
    sim_end();
    return 0;
}
