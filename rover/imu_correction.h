#ifndef IMU_CORRECTION_H
#define IMU_CORRECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "bmi270_port.h"
#include "kalman_wrapper.h"
#include "bmi2.h"
#include "bmi270.h"


// ------------------------ Kalman Movement Implementation ---------------------------
#define DEFAULT_MOTOR_SPEED         (128/2)     // can change this later
typedef enum {
    STOPPED,
    SLOW,
    MED,
    FAST
} SPEED;

typedef enum {
    NONE,
    LOW,
    MODERATE,
    HIGH
} TILT_ANGLE;

typedef enum {
    PITCH_DEADZONE,
    FORWARDS,
    BACKWARDS
} PITCH_DIRECTION;

typedef enum {
    ROLL_DEADZONE,
    LEFT,
    RIGHT
} ROLL_DIRECTION;

typedef struct {
    TILT_ANGLE tilt;
    SPEED motion_speed;
    PITCH_DIRECTION pitch_dir;
    ROLL_DIRECTION roll_dir;
    float tilt_angle_deg;
} mapping_t;
// -------------------------------------------------------------------

// getter functions
bool imu_correction_is_fault(void);
bool imu_correction_is_critical(void);
bool imu_correction_is_warning(void);
bool imu_correction_is_stall(void);
bool imu_correction_is_unintended_motion(void);
const mapping_t* imu_correction_get_map(void);

// init and update
int imu_correction_init(struct bmi2_dev* dev);
int imu_correction_update(struct bmi2_dev* dev);

// heading reference management
void imu_correction_set_heading_reference(void);
void imu_correction_clear_heading_reference(void);

// motor state
// void imu_correction_set_motor_state(bool running);

// terrain handler
int imu_correction_handle_terrain(const mapping_t* map);

// motion check
int imu_correction_check_motion(void);

// correction application
int imu_correction_apply(void);

#endif
