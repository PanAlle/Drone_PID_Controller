#ifndef DRONE_SIMULATION_H
#define DRONE_SIMULATION_H

#define TIME_STEP                0.01f // [s]
#define DRONE_MAX_ACCELERATION  13.0f  // [m/s^2]
#define EARTH_ACCELERATION      -9.81f // [m/s^2]

/// Start the simulation and create a log file which contains flight data.
/// File contains: time, height, speed, acceleration, user_data
/// Matlab example for plotting the height over time:
///   flight_data = csvread("flight.log",1);
///   plot(flight_data(:,1), flight_data(:,2)); % plot height
void sim_start(const char *filename);

/// End the simulation.
void sim_end(void);

/// Increase the simulation time by a timer tick.
/// Returns current time in seconds.
float sim_advance_time(void);

/// Returns the vehicle's height over ground in meters.
float sim_get_height(void);

#define MAX_ACCELERATION ((DRONE_MAX_ACCELERATION + EARTH_ACCELERATION) / (-1 * EARTH_ACCELERATION))
#define MIN_ACCELERATION (-1.0f)
/// Set the vertical acceleration of the vehicles (must be in range
/// MIN_ACCELERATION to MAX_ACCELERATION, a value of 0 results in a constant
/// drone velocity).
void sim_set_acceleration(float acceleration);

/// Set optional user data which will be logged in the flight data file.
/// Note: User data can be used for logging the target height.
void sim_set_user_data(float user_data);


/// Example trajectory for a simulation in 0 < t < 30 seconds

#define move_linear(t,t1,t2,z1,z2) ((z1) + ((t)-(t1))*((z2)-(z1))/((t2)-(t1)))
#define move_linear_in_interval(t,t1,t2,z1,z2)      \
if((t) >= (t1) && (t) < (t2)){                      \
    return move_linear((t),(t1),(t2),(z1),(z2)); }

/// Returns the target height for a given point in time.
/// Use the function without the leading underscore.
static inline double _sim_get_example_trajectory(float time)
{
    move_linear_in_interval(time,  0.0f,  0.5f, 0.0f, 0.0f);
    move_linear_in_interval(time,  0.5f,  6.0f, 0.0f, 5.0f);
    move_linear_in_interval(time,  6.0f, 15.0f, 5.0f, 5.0f);
    move_linear_in_interval(time, 15.0f, 24.5f, 5.0f, 0.5f);
    move_linear_in_interval(time, 24.5f, 29.0f, 0.5f, 0.0f);
    return 0.0f;
}
// This is an ugly workaround to silence a buggy warning in the SimpleC compiler.
#define sim_get_example_trajectory(t) ((float)_sim_get_example_trajectory(t))

#endif //DRONE_SIMULATION_H
