#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "imu_correction.h"
#include "rover.h"


// --------------------------- globals -------------------------------
#define NUM_SAMPLES 100

#define BRAKE_SPEED                 (64/2)    // this is the breaking speed (can change later based on tuning from testing)
#define TERRAIN_SLOW_SPEED          (64/2)
#define TERRAIN_MODERATE_SPEED      (48/2)
#define TERRAIN_MIN_SPEED           (32/2)

// thresholds
#define STALL_MAG_THRESHOLD                  1.08f      // can change to 1.05 later (need to tune based on testing)
#define MOTION_MAG_THRESHOLD                 1.18f      // can change to 1.15 later (need to tune based on testing)
#define STALL_TIME_THRESHOLD_US              500000     // threshold for stall time
#define UNINTENDED_MOTION_TIME_THRESHOLD_US  200000     // sliding needs a faster response
#define ENCODER_DELTA_THRESHOLD              10         // can change later

// deadband, weight, max correction limits
#define YAW_ERROR_DEADBAND 0.5f        // placeholder value, adjust as needed
#define GYRO_RATE_DEADBAND 2.0f        // placeholder value, adjust as needed
#define YAW_WEIGHT         0.7f        // placeholder weight, adjust as needed
#define RATE_WEIGHT        0.3f        // placeholder weight, adjust as needed.
#define MAX_CORRECTION     10.0f       // placeholder max correction, adjust as needed.

// type variables
static float yaw_reference = 0.0f;
static float yaw_error = 0.0f;
static float gyr_z_bias = 0.0f;
static float gyro_z_rate_corrected = 0.0f;
static int64_t start_dt = 0;
static double mag_accel = 0.0;
static float ax = 0.0f;


// added roll and pitch offsets because the rover favors one side more than the other because of the arm
static float roll_offset = 0.00f;
static float pitch_offset = 0.00f;


static mapping_t map;

// flags for motion correction
static bool ready_flag = false;
// static bool terrain_flag = false;
static bool warning_flag = false;
static bool critical_flag = false;
static bool fault_flag = false;

// used for imu_correction_check_motion()
// static bool motor_running = false;
static bool unintended_motion_flag = false;
static bool stall_flag = false;
// -------------------------------------------------------------------


typedef struct {
    float heading_reference;
    float heading_error_accum;
    float last_correction;
    bool active;
} imu_correction_state;

static imu_correction_state correction_state;


// ------------------------ Getter Functions ----------------------------
bool imu_correction_is_fault(void){
    return fault_flag;
}

bool imu_correction_is_critical(void){
    return critical_flag;
}

bool imu_correction_is_warning(void){
    return warning_flag;
}

bool imu_correction_is_stall(void){
    return stall_flag;
}

bool imu_correction_is_unintended_motion(void){
    return unintended_motion_flag;
}

const mapping_t* imu_correction_get_map(void){
    return &map;
}
// ------------------------------------------------------------------------


// void imu_correction_set_motor_state(bool running) {
//     motor_running = running;
// }


// function for calculating mag acceleration based on ax, ay, az, and dt
static double magnitude_acceleration(float ax, float ay, float az){
    return sqrt((double)((ax*ax) + (ay*ay) + (az*az)));
}


static void get_direction_and_speed(float roll, float pitch, double mag_acceleration, mapping_t *map){

    float roll_deg = roll * (180 / M_PI);
    float pitch_deg = pitch * (180 / M_PI);

    if (pitch_deg < 0.0f){
        map->pitch_dir = FORWARDS;
    }
    else if (pitch_deg > 0.0f){
        map->pitch_dir = BACKWARDS;
    }
    else {
        map->pitch_dir = PITCH_DEADZONE;
    }

    if (roll_deg < 0.f){
        map->roll_dir = LEFT;
    }
    else if (roll_deg > 0.0f){
        map->roll_dir = RIGHT;
    }
    else {
        map->roll_dir = ROLL_DEADZONE;
    }

    float abs_roll_deg = fabsf(roll_deg);
    float abs_pitch_deg = fabsf(pitch_deg);

    float tilt_angle_deg = ((abs_pitch_deg > abs_roll_deg) ? abs_pitch_deg : abs_roll_deg);
    map->tilt_angle_deg = tilt_angle_deg;

    if (tilt_angle_deg < 10.00){
        map->tilt = NONE;
    }
    else if (tilt_angle_deg < 20.00){
        map->tilt = LOW;
    }
    else if (tilt_angle_deg < 35.00){
        map->tilt = MODERATE;
    }
    else {
        map->tilt = HIGH;
    }

    if (mag_acceleration <= 1.05 ){
        map->motion_speed = STOPPED;
    }
    else if (mag_acceleration > 1.05 && mag_acceleration <= 1.20){
        map->motion_speed = SLOW;
    }
    else if (mag_acceleration > 1.20 && mag_acceleration <= 1.40){
        map->motion_speed = MED;
    }
    else {
        map->motion_speed = FAST;
    }
}


static void new_set_all_motor_speeds(int speed){


    motor_set_speed(FRW, -speed);
    motor_set_speed(FLW, speed);
    motor_set_speed(MRW, -speed);
    motor_set_speed(MLW, speed);
    motor_set_speed(RRW, -speed);
    motor_set_speed(RLW, speed);
}


static void new_rover_stop(void){
    motor_set_speed(FRW, 0);
    motor_set_speed(FLW, 0);
    motor_set_speed(MRW, 0);
    motor_set_speed(MLW, 0);
    motor_set_speed(RRW, 0);
    motor_set_speed(RLW, 0);
}


void imu_correction_set_heading_reference(){
    float roll, pitch, yaw;
    kalman_get_attitude(&roll, &pitch, &yaw);

    // DEBUG
    printf("SET_HEADING | roll: %.3f pitch: %.3f yaw: %.3f\n", roll, pitch, yaw);

    // store heading reference
    correction_state.heading_reference = yaw;
    yaw_reference = yaw;
   
    // reset correction state
    correction_state.heading_error_accum = 0.0f;
    correction_state.last_correction = 0.0f;

    // state active
    correction_state.active = true;
}


void imu_correction_clear_heading_reference(){
    correction_state.active = false;
    correction_state.heading_reference = 0.0f;
    correction_state.heading_error_accum = 0.0f;
    correction_state.last_correction = 0.0f;
}


int imu_correction_init(struct bmi2_dev* dev){

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    start_dt = (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
    int64_t end_dt;
    float delta_dt;

    // float ay, az;
    float gx, gy, gz;

    int8_t rslt;

    int i = 0;
    for (; i < NUM_SAMPLES; i++){

        // get time variables for loop iteration
        struct timespec ts_end;
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        end_dt = (int64_t)ts_end.tv_sec * 1000000LL + ts_end.tv_nsec / 1000LL;
        delta_dt = (float) (end_dt - start_dt) / 1000000.0f;

        // getting accel and gyr
        struct bmi2_sens_axes_data acc, gyr;
        rslt = bmi270_kria_read_ag(dev, &acc, &gyr);

        // check result of reading the registers on IMU
        if (rslt == BMI2_OK){

            ax = (float) (acc.x / (32768.0f / 4.0f));
            float ay = (float) (acc.y / (32768.0f / 4.0f));
            float az = (float) (acc.z / (32768.0f / 4.0f));

            gx = (float) (gyr.x / (32768.0f / 2000.0f));
            gy = (float) (gyr.y / (32768.0f / 2000.0f));
            gz = (float) (gyr.z / (32768.0f / 2000.0f));
            gyr_z_bias += (float) (gyr.z / (32768.0f / 2000.0f));

            if (i == 0 || i == 49 || i == 99){
                float r, p, yw;
                kalman_get_attitude(&r, &p, &yw);
                printf("INIT sample %d | roll: %.3f pitch: %.3f yaw: %.3f dt: %.6f\n", i, r, p, yw, delta_dt);
            }

            // update the kalman filter with data from IMU
            if (delta_dt <= 0.0f || delta_dt > 1.0f){
                printf("WARNING: bad delta_dt in init: %.6f, skipping kalman_update\n", delta_dt);
                start_dt = end_dt;
                continue;
            }
            kalman_update(ax, ay, -az, gx, gy, gz, delta_dt);
            bmi270_delay_us(3333, NULL);
        }
        else {
            printf("The result of bmi270_kria_read_ag() was not BMI2_OK\n");
            return -1;
        }

        // reassign the start time
        start_dt = end_dt;
    }

    gyr_z_bias /= i;

    // want to get the attitude so we can update the yaw_reference, roll and pitch offsets global variable
    float roll, pitch, yaw;
    kalman_get_attitude(&roll, &pitch, &yaw);

    printf("INIT END | roll: %.3f pitch: %.3f yaw: %.3f\n", roll, pitch, yaw);

    yaw_reference = yaw;
    roll_offset = roll;
    pitch_offset = pitch;

    // set the ready flag to true and return
    ready_flag = true;

    return 0;
}

// static void set_all_motor_speeds(int speed){


//     // set all the motors based on the speed parameter
//     motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
//     motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
//     motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
//     motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
//     motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
//     motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
// }


int imu_correction_update(struct bmi2_dev* dev){

    // check to see if we are ready (if not the init was unsuccessful)
    if (ready_flag == false){
        printf("ready flag is not set in imu_correction_update()\n");
        return -1;
    }

    warning_flag = false;
    critical_flag = false;
    // terrain_flag = false;

    // computing the variables for time (start, end, and delta)
    struct timespec ts_end;
    clock_gettime(CLOCK_MONOTONIC, &ts_end);
    int64_t end_dt = (int64_t)ts_end.tv_sec * 1000000LL + ts_end.tv_nsec / 1000LL;
    float delta_dt = (float) (end_dt - start_dt) / 1000000.0f;
    start_dt = end_dt;

    // float ay, az;
    // float gx, gy, gz;

    // get data for
    struct bmi2_sens_axes_data acc, gyr;
    int8_t rslt = bmi270_kria_read_ag(dev, &acc, &gyr);

    if (rslt == BMI2_OK){

        ax = (float) (acc.x / (32768.0f / 4.0f));       // this one is global
        float ay = (float) (acc.y / (32768.0f / 4.0f));
        float az = (float) (acc.z / (32768.0f / 4.0f));

        float gx = (float) (gyr.x / (32768.0f / 2000.0f));
        float gy = (float) (gyr.y / (32768.0f / 2000.0f));
        float gz = (float) (gyr.z / (32768.0f / 2000.0f));

        // update the kalman filter
        // float delta_dt = (float)(end_dt - start_dt) / 1000000.0f;
        start_dt = end_dt;

        if (delta_dt <= 0.0f || delta_dt > 1.0f){
            printf("WARNING: bad delta_dt in update: %.6f\n", delta_dt);
            return 0;
        }
        kalman_update(ax, ay, -az, gx, gy, gz, delta_dt);

        // get roll, pitch, and yaw
        float roll, pitch, yaw;
        kalman_get_attitude(&roll, &pitch, &yaw);

        // subtraction the offsets from the roll and pitch
        roll = roll - roll_offset;
        pitch = pitch - pitch_offset;

        // calculating the distance moved (global variable)
        mag_accel = magnitude_acceleration(ax, ay, az);

        get_direction_and_speed(roll, pitch, mag_accel, &map);




        // checking to see if the rover is stopped
        if (map.motion_speed == STOPPED){
            yaw_error = yaw - yaw_reference;
            gyro_z_rate_corrected = gz - gyr_z_bias;
            return 0;
        }

        switch (map.tilt){

            case NONE:
                break;

            case LOW:
                warning_flag = true;
                return 0;

            case MODERATE:
                critical_flag = true;
                new_set_all_motor_speeds(TERRAIN_SLOW_SPEED);
                // motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, TERRAIN_SLOW_SPEED);
                // motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, TERRAIN_SLOW_SPEED);
                // motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, TERRAIN_SLOW_SPEED);
                // motor_set_speed(MOTOR_REAR_LEFT_WHEEL, TERRAIN_SLOW_SPEED);
                // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, TERRAIN_SLOW_SPEED);
                // motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, TERRAIN_SLOW_SPEED);
                return 0;

            case HIGH:
                fault_flag = true;
                new_rover_stop();          
                // rover_stop();    // can't call rover_stop() (does not work)
                return -1;

            default:
                fault_flag = true;
                printf("In default case statement for imu_correction_update()\n");
                new_rover_stop();           // untested
                // rover_stop();    // can't call rover_stop() (does not work)
                return -1;
        }

        yaw_error = yaw - yaw_reference;
        gyro_z_rate_corrected = gz - gyr_z_bias;
    }
    else {
        // if there was an error reading from the IMU, set the fault flag
        printf("There was an error reading from bmi270_kria_read_ag() in imu_correction_update()\n");
        new_rover_stop();
        // rover_stop();    // can't call rover_stop() (does not work)

        fault_flag = true;
        return -1;
    }

    return 0;
}


int imu_correction_handle_terrain(const mapping_t* map){

    // check roll first (are we moving?)
    if (map->roll_dir != ROLL_DEADZONE){

        // if the tilt angle is high there is an issue (maybe)
        if (map->tilt == HIGH){
            fault_flag = true;
            new_rover_stop();
            // rover_stop();    // can't call rover_stop() (does not work)
            return -1;
        }
        else if (map->tilt == MODERATE){    // could be fine, slow down

            float fraction = (map->tilt_angle_deg - 20.0f) / (35.0f - 20.0f);
            int speed = (int) ((float)DEFAULT_MOTOR_SPEED + (TERRAIN_MODERATE_SPEED - (float)DEFAULT_MOTOR_SPEED) * fraction);

            new_set_all_motor_speeds(speed);


            // motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);

            // am unable to use this function because the calibration does not work...
            // rover_steer_forward();  
        }
        else if (map->tilt == LOW){     // this is OK

            float fraction = (map->tilt_angle_deg - 10.0f) / (20.0f - 10.0f);
            int speed = (int) ((float)DEFAULT_MOTOR_SPEED + (TERRAIN_SLOW_SPEED - (float)DEFAULT_MOTOR_SPEED) * fraction);
            new_set_all_motor_speeds(speed);          
            // motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
        }
        else {
            printf("Unexpected combination\n");
            fault_flag = true;
            return -1;
        }
    }

    // checking the pitch now
    else if (map->pitch_dir != PITCH_DEADZONE){

        if (map->tilt == HIGH){
            // motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, TERRAIN_MIN_SPEED);
            // motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, TERRAIN_MIN_SPEED);
            // motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, TERRAIN_MIN_SPEED);
            // motor_set_speed(MOTOR_REAR_LEFT_WHEEL, TERRAIN_MIN_SPEED);
            // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, TERRAIN_MIN_SPEED);
            // motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, TERRAIN_MIN_SPEED);


            new_set_all_motor_speeds(TERRAIN_MIN_SPEED);
        }
        else if (map->tilt == MODERATE){
            // slow down all six wheels            
            float fraction = (map->tilt_angle_deg - 20.0f) / (35.0f - 20.0f);
            int speed = (int) ((float)DEFAULT_MOTOR_SPEED + (TERRAIN_MODERATE_SPEED - (float)DEFAULT_MOTOR_SPEED) * fraction);

            new_set_all_motor_speeds(speed);
            // motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
        }
        else if (map->tilt == LOW){

            float fraction = (map->tilt_angle_deg - 10.0f) / (20.0f - 10.0f);
            int speed = (int) ((float)DEFAULT_MOTOR_SPEED + (TERRAIN_SLOW_SPEED - (float)DEFAULT_MOTOR_SPEED) * fraction);

            new_set_all_motor_speeds(speed);
            // motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            // motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            // motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
        }
        else {
            printf("Unexpected combination\n");
            fault_flag = true;
            return -1;
        }
    }
    else {
        printf("Warning: Pitch and Roll are in a deadzone (flat land), should not be called\n");
        fault_flag = true;
        return -1;
    }

    float roll, pitch, yaw;
    kalman_get_attitude(&roll, &pitch, &yaw);
    yaw_reference = yaw;

    return 0;
}


static float clamp(float val, float min, float max){
    if (val < min) return min;
    if (val > max) return max;
    return val;
}


int imu_correction_apply(){
    if (!correction_state.active){
        return 0;
    }

    float yaw_err = yaw_error;
    float gyro_z_rate_err = gyro_z_rate_corrected;

    if ((fabsf(yaw_err) < YAW_ERROR_DEADBAND) && (fabsf(gyro_z_rate_err) < GYRO_RATE_DEADBAND)) {
        correction_state.last_correction = 0;
        return 0;
    }

    float weighted_correction = yaw_err * YAW_WEIGHT + gyro_z_rate_err * RATE_WEIGHT;
    weighted_correction = clamp(weighted_correction, -MAX_CORRECTION, MAX_CORRECTION);

    printf("IMU CORRECTION | YAW_ERR: %.3f | GYRO_Z: %.3f | CORRECTION: %.3f\n", yaw_err, gyro_z_rate_err, weighted_correction);

    if (weighted_correction > 0){
        // rover drifting right — slow left side to correct back toward straight
        motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, -DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, -DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, -DEFAULT_MOTOR_SPEED);
        // reduced speed on the right motors
        motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, DEFAULT_MOTOR_SPEED - abs((int)weighted_correction));
        motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, DEFAULT_MOTOR_SPEED - abs((int)weighted_correction));
        motor_set_speed(MOTOR_REAR_LEFT_WHEEL, DEFAULT_MOTOR_SPEED - abs((int)weighted_correction));
    }
    else if (weighted_correction < 0){
        // rover drifting left — slow right side to correct back toward straight
        motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_REAR_LEFT_WHEEL, DEFAULT_MOTOR_SPEED);
        // reduced speed on the right motors
        motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, -(DEFAULT_MOTOR_SPEED - abs((int)weighted_correction)));
        motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, -(DEFAULT_MOTOR_SPEED - abs((int)weighted_correction)));
        motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, -(DEFAULT_MOTOR_SPEED - abs((int)weighted_correction)));
    }

    correction_state.last_correction = weighted_correction;
    return 1;
}


int imu_correction_check_motion(){

    if (ready_flag == false){
        return -1;
    }

    static int64_t stall_start_us = 0;
    static int64_t unintended_motion_start_us = 0;

    // to store the position read from wheel encoders
    static int64_t prev_pos_frw = 0;
    static int64_t prev_pos_flw = 0;
    static int64_t prev_pos_mrw = 0;
    static int64_t prev_pos_mlw = 0;
    static int64_t prev_pos_rrw = 0;
    static int64_t prev_pos_rlw = 0;

    // flag for first call
    static bool first_call = true;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t current_us = (int64_t) ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;


    // get motor positions on each wheel:
    int64_t curr_frw = motor_get_position(MOTOR_FRONT_RIGHT_WHEEL);
    int64_t curr_flw = motor_get_position(MOTOR_FRONT_LEFT_WHEEL);
    int64_t curr_mrw = motor_get_position(MOTOR_MIDDLE_RIGHT_WHEEL);
    int64_t curr_mlw = motor_get_position(MOTOR_MIDDLE_LEFT_WHEEL);
    int64_t curr_rrw = motor_get_position(MOTOR_REAR_RIGHT_WHEEL);
    int64_t curr_rlw = motor_get_position(MOTOR_REAR_LEFT_WHEEL);

    // guard for first time the function is called
    if (first_call){
        first_call = false;
        prev_pos_frw = curr_frw;
        prev_pos_flw = curr_flw;
        prev_pos_mrw = curr_mrw;
        prev_pos_mlw = curr_mlw;
        prev_pos_rrw = curr_rrw;
        prev_pos_rlw = curr_rlw;
        return 0;
    }

    // compute the delta in curr - prev
    int64_t delta_frw = curr_frw - prev_pos_frw;
    int64_t delta_flw = curr_flw - prev_pos_flw;
    int64_t delta_mrw = curr_mrw - prev_pos_mrw;
    int64_t delta_mlw = curr_mlw - prev_pos_mlw;
    int64_t delta_rrw = curr_rrw - prev_pos_rrw;
    int64_t delta_rlw = curr_rlw - prev_pos_rlw;

    // overall wheel estimate
    int64_t avg_wheel_dist = (delta_frw + delta_flw + delta_mrw + delta_mlw + delta_rrw + delta_rlw) / 6;
    int64_t avg_wheel_delta = llabs(avg_wheel_dist);

    // update the prev wheels
    prev_pos_frw = curr_frw;
    prev_pos_flw = curr_flw;
    prev_pos_mrw = curr_mrw;
    prev_pos_mlw = curr_mlw;
    prev_pos_rrw = curr_rrw;
    prev_pos_rlw = curr_rlw;

    // if the rover is moving
    if (avg_wheel_delta > ENCODER_DELTA_THRESHOLD){

        if (mag_accel < STALL_MAG_THRESHOLD){

            if (stall_start_us == 0){
                stall_start_us = current_us;
            }
            else {
                int64_t elapsed_time = current_us - stall_start_us;

                if (elapsed_time > STALL_TIME_THRESHOLD_US){
                    stall_flag = true;
                    // rover_reverse(BRAKE_SPEED);
                    new_set_all_motor_speeds(-BRAKE_SPEED);
                    bmi270_delay_us(200000, NULL);
                    new_rover_stop();
                    // rover_stop();    // can't call rover_stop() (does not work)
                    stall_start_us = 0;
                    return -1;
                }
            }
        }
        else {
            stall_start_us = 0;
        }
    }
    else {

        if (mag_accel > MOTION_MAG_THRESHOLD){
            if (unintended_motion_start_us == 0){
                unintended_motion_start_us = current_us;
            }
            else {
                int64_t elapsed_time = current_us - unintended_motion_start_us;
                if (elapsed_time > UNINTENDED_MOTION_TIME_THRESHOLD_US){
                    unintended_motion_flag = true;

                    // if we are moving forward
                    if (ax > 0.0f){
                        // rover_reverse(BRAKE_SPEED);
                        new_set_all_motor_speeds(-BRAKE_SPEED);
                        bmi270_delay_us(200000, NULL);
                        new_rover_stop();
                        // rover_stop();    // can't call rover_stop() (does not work)
                    }
                    else {
                        // rover_forward(BRAKE_SPEED);
                        new_set_all_motor_speeds(BRAKE_SPEED);
                        bmi270_delay_us(200000, NULL);
                        new_rover_stop();
                        // rover_stop();    // can't call rover_stop() (does not work)
                    }

                    unintended_motion_start_us = 0;
                    return -1;
                }
            }
        }
        else {
            unintended_motion_start_us = 0;
            // rover is correctly stopped, no unintended motion
        }
    }

    return 0;
}
