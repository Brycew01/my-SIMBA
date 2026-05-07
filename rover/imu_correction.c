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

#define DEFAULT_MOTOR_SPEED 128
#define BRAKE_SPEED 64
#define TERRAIN_SLOW_SPEED 64
#define TERRAIN_MODERATE_SPEED 48
#define TERRAIN_MIN_SPEED 32

// thresholds
#define STALL_MAG_THRESHOLD 1.08f
#define MOTION_MAG_THRESHOLD 1.18f
#define STALL_TIME_THRESHOLD_US  500000
#define UNINTENDED_MOTION_TIME_THRESHOLD_US  200000

// deadband, weight, max correction limits
#define YAW_ERROR_DEADBAND 0.00f
#define GYRO_RATE_DEADBAND 0.00f
#define YAW_WEIGHT 0.0f
#define RATE_WEIGHT 0.0f
#define MAX_CORRECTION 0.0f

// type variables
static float yaw_reference = 0.0f;
static float yaw_error = 0.0f;
static float gyr_z_bias = 0.0f;
static float gyro_z_rate_corrected = 0.0f;
static int64_t start_dt = 0;
static double mag_dist = 0.0;
static float ax = 0.0f;
static mapping_t map;

// flags for motion correction
static bool ready_flag = false;
static bool terrain_flag = false;
static bool warning_flag = false;
static bool critical_flag = false;
static bool fault_flag = false;

// used for imu_correction_check_motion()
static bool motor_running = false;
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


void imu_correction_set_motor_state(bool running) {
    motor_running = running;
}


static double magnitude_distance(float ax, float ay, float az){
    return sqrt((double)((ax*ax) + (ay*ay) + (az*az)));
}


static void get_direction_and_speed(float roll, float pitch, double mag_distance, mapping_t *map){

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

    if (mag_distance <= 1.05 ){
        map->motion_speed = STOPPED;
    }
    else if (mag_distance > 1.05 && mag_distance <= 1.20){
        map->motion_speed = SLOW;
    }
    else if (mag_distance > 1.20 && mag_distance <= 1.40){
        map->motion_speed = MED;
    }
    else {
        map->motion_speed = FAST;
    }
}


void imu_correction_set_heading_reference(){
    float roll, pitch, yaw;
    kalman_get_attitude(&roll, &pitch, &yaw);

    correction_state.heading_reference = yaw;
    correction_state.heading_error_accum = 0.0f;
    correction_state.last_correction = 0.0f;
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

    float ay, az;
    float gx, gy, gz;

    int8_t rslt;

    int i = 0;
    for (; i < NUM_SAMPLES; i++){

        struct bmi2_sens_axes_data acc, gyr;
        rslt = bmi270_kria_read_ag(dev, &acc, &gyr);

        struct timespec ts_end;
        clock_gettime(CLOCK_MONOTONIC, &ts_end);
        end_dt = (int64_t)ts_end.tv_sec * 1000000LL + ts_end.tv_nsec / 1000LL;
        delta_dt = (float) (end_dt - start_dt) / 1000000.0f;

        if (rslt == BMI2_OK){

            ax = (float) (acc.x / (32768.0f / 4.0f));
            ay = (float) (acc.y / (32768.0f / 4.0f));
            az = (float) (acc.z / (32768.0f / 4.0f));

            gx = (float) (gyr.x / (32768.0f / 2000.0f));
            gy = (float) (gyr.y / (32768.0f / 2000.0f));
            gz = (float) (gyr.z / (32768.0f / 2000.0f));
            gyr_z_bias += (float) (gyr.z / (32768.0f / 2000.0f));

            kalman_update(ax, ay, -az, gx, gy, gz, delta_dt);
            bmi270_delay_us(3333, NULL);
        }
        else {
            printf("The result of bmi270_kria_read_ag() was not BMI2_OK\n");
            return -1;
        }

        start_dt = end_dt;
    }

    gyr_z_bias /= i;

    float roll, pitch, yaw;
    kalman_get_attitude(&roll, &pitch, &yaw);

    yaw_reference = yaw;
    ready_flag = true;

    return 0;
}


int imu_correction_update(struct bmi2_dev* dev){

    if (ready_flag == false){
        return -1;
    }

    warning_flag = false;
    critical_flag = false;
    terrain_flag = false;

    struct timespec ts_end;
    clock_gettime(CLOCK_MONOTONIC, &ts_end);
    int64_t end_dt = (int64_t)ts_end.tv_sec * 1000000LL + ts_end.tv_nsec / 1000LL;
    float delta_dt = (float) (end_dt - start_dt) / 1000000.0f;
    start_dt = end_dt;

    float ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;

    struct bmi2_sens_axes_data acc, gyr;
    int8_t rslt = bmi270_kria_read_ag(dev, &acc, &gyr);

    if (rslt == BMI2_OK){

        ax = (float) (acc.x / (32768.0f / 4.0f));
        ay = (float) (acc.y / (32768.0f / 4.0f));
        az = (float) (acc.z / (32768.0f / 4.0f));

        gx = (float) (gyr.x / (32768.0f / 2000.0f));
        gy = (float) (gyr.y / (32768.0f / 2000.0f));
        gz = (float) (gyr.z / (32768.0f / 2000.0f));

        kalman_update(ax, ay, -az, gx, gy, gz, delta_dt);
        kalman_get_attitude(&roll, &pitch, &yaw);

        mag_dist = magnitude_distance(ax, ay, az);

        get_direction_and_speed(roll, pitch, mag_dist, &map);
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
                motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, TERRAIN_SLOW_SPEED);
                motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, TERRAIN_SLOW_SPEED);
                motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, TERRAIN_SLOW_SPEED);
                motor_set_speed(MOTOR_REAR_LEFT_WHEEL, TERRAIN_SLOW_SPEED);
                motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, TERRAIN_SLOW_SPEED);
                motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, TERRAIN_SLOW_SPEED);
                return 0;

            case HIGH:
                fault_flag = true;
                rover_stop();
                return -1;

            default:
                fault_flag = true;
                printf("In default case statement for imu_correction_update()\n");
                rover_stop();
                return -1;
        }

        yaw_error = yaw - yaw_reference;
        gyro_z_rate_corrected = gz - gyr_z_bias;
    }
    else {
        printf("There was an error reading from bmi270_kria_read_ag() in imu_correction_update()\n");
        rover_stop();
        fault_flag = true;
        return -1;
    }

    return 0;
}


int imu_correction_handle_terrain(const mapping_t* map){

    if (map->roll_dir != ROLL_DEADZONE){

        if (map->tilt == HIGH){
            fault_flag = true;
            rover_stop();
            return -1;
        }
        else if (map->tilt == MODERATE){

            float fraction = (map->tilt_angle_deg - 20.0f) / (35.0f - 20.0f);
            int speed = (int) (128.0f + (TERRAIN_MODERATE_SPEED - 128.0f) * fraction);

            motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);

            rover_steer_forward();
        }
        else if (map->tilt == LOW){

            float fraction = (map->tilt_angle_deg - 10.0f) / (20.0f - 10.0f);
            int speed = (int) (128.0f + (TERRAIN_SLOW_SPEED - 128.0f) * fraction);

            motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
        }
        else {
            printf("Unexpected combination\n");
            fault_flag = true;
            return -1;
        }
    }

    else if (map->pitch_dir != PITCH_DEADZONE){

        if (map->tilt == HIGH){
            motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, TERRAIN_MIN_SPEED);
            motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, TERRAIN_MIN_SPEED);
            motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, TERRAIN_MIN_SPEED);
            motor_set_speed(MOTOR_REAR_LEFT_WHEEL, TERRAIN_MIN_SPEED);
            motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, TERRAIN_MIN_SPEED);
            motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, TERRAIN_MIN_SPEED);
        }
        else if (map->tilt == MODERATE){

            float fraction = (map->tilt_angle_deg - 20.0f) / (35.0f - 20.0f);
            int speed = (int) (128.0f + (TERRAIN_MODERATE_SPEED - 128.0f) * fraction);

            motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
        }
        else if (map->tilt == LOW){

            float fraction = (map->tilt_angle_deg - 10.0f) / (20.0f - 10.0f);
            int speed = (int) (128.0f + (TERRAIN_SLOW_SPEED - 128.0f) * fraction);

            motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, speed);
            motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_REAR_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, speed);
            motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, speed);
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

    int weighted_correction = yaw_err * YAW_WEIGHT + gyro_z_rate_err * RATE_WEIGHT;
    weighted_correction = clamp(weighted_correction, -MAX_CORRECTION, MAX_CORRECTION);

    if (weighted_correction > 0){
        motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, DEFAULT_MOTOR_SPEED - abs(weighted_correction));
        motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, DEFAULT_MOTOR_SPEED - abs(weighted_correction));
        motor_set_speed(MOTOR_REAR_LEFT_WHEEL, DEFAULT_MOTOR_SPEED - abs(weighted_correction));
    }
    else if (weighted_correction < 0){
        motor_set_speed(MOTOR_FRONT_LEFT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_REAR_LEFT_WHEEL, DEFAULT_MOTOR_SPEED);
        motor_set_speed(MOTOR_FRONT_RIGHT_WHEEL, DEFAULT_MOTOR_SPEED - abs(weighted_correction));
        motor_set_speed(MOTOR_MIDDLE_RIGHT_WHEEL, DEFAULT_MOTOR_SPEED - abs(weighted_correction));
        motor_set_speed(MOTOR_REAR_RIGHT_WHEEL, DEFAULT_MOTOR_SPEED - abs(weighted_correction));
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

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t current_us = (int64_t) ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;

    if (motor_running == true){

        if (mag_dist < STALL_MAG_THRESHOLD){

            if (stall_start_us == 0){
                stall_start_us = current_us;
            }
            else {
                int64_t elapsed_time = current_us - stall_start_us;

                if (elapsed_time > STALL_TIME_THRESHOLD_US){
                    stall_flag = true;
                    rover_reverse(BRAKE_SPEED);
                    bmi270_delay_us(200000, NULL);
                    rover_stop();
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

        if (mag_dist > MOTION_MAG_THRESHOLD){
            if (unintended_motion_start_us == 0){
                unintended_motion_start_us = current_us;
            }
            else {
                int64_t elapsed_time = current_us - unintended_motion_start_us;
                if (elapsed_time > UNINTENDED_MOTION_TIME_THRESHOLD_US){
                    unintended_motion_flag = true;

                    if (ax > 0.0f){
                        rover_reverse(BRAKE_SPEED);
                        bmi270_delay_us(200000, NULL);
                        rover_stop();
                    }
                    else {
                        rover_forward(BRAKE_SPEED);
                        bmi270_delay_us(200000, NULL);
                        rover_stop();
                    }

                    unintended_motion_start_us = 0;
                    return -1;
                }
            }
        }
        else {
            unintended_motion_start_us = 0;
        }
    }

    return 0;
}
