// TEST MMIO GPIO CONTROL
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
// #include <unistd.h>


#include "rover.h"
#include "servo.h"
#include "imu_correction.h"
#include "kalman_wrapper.h"
#include "bmi270_port.h"


// globals
// #define DEFAULT_MOTOR_SPEED 128
static bool done = false;


void sigint_handler(int signum) {
  (void)signum;
  printf("SIGINT caught\n");
  done = true;
}




// ------------- added some wrapper functions for simba movement ------------------------
static void simba_forward(int speed){
 
  // imu_correction_set_motor_state(true);
  imu_correction_set_heading_reference();   // gets a snapshot of yaw as the reference to maintain during straight driving
  rover_forward(speed);    
}


// static void simba_reverse(int speed){


//   imu_correction_set_motor_state(true);
//   imu_correction_set_heading_reference();   // gets a snapshot of yaw as the reference to maintain during straight driving
//   rover_reverse(speed);
// }


static void simba_stop(void){


  // imu_correction_set_motor_state(false);
  imu_correction_clear_heading_reference();
  //rover_stop(); //unexpected behavior from this function atm
  motor_set_speed(FRW, 0);
  motor_set_speed(RRW, 0);
  motor_set_speed(FLW, 0);
  motor_set_speed(RLW, 0);
  motor_set_speed(MRW, 0);
  motor_set_speed(MLW, 0);
}


// static void simba_point_turn_cw(int speed){


//   imu_correction_set_motor_state(true);
//   imu_correction_clear_heading_reference();
//   rover_pointTurn_CW(speed);
// }


// static void simba_point_turn_ccw(int speed){


//   imu_correction_set_motor_state(true);
//   imu_correction_clear_heading_reference();
//   rover_pointTurn_CCW(speed);
// }
// --------------------------------------------------------------------------


int main() {
  // Configure signal handler
  signal(SIGINT, sigint_handler);


  // opening the fd and checking for error
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) {
      perror("open /dev/mem");
      return 1;
  }


  // base address in memory of the IMU
  iic_base = (volatile uint32_t*)mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, AXI_IIC_BASE);


  // checking the return type    
  if (iic_base == MAP_FAILED) {
      perror("mmap");
      return 1;
  }


  bmi270_i2c_ctx_t ctx = { .port = 0, .addr = 0x68};   // context struct - holds HW specific info (how do I configure this?)
  struct bmi2_dev dev = {0};    // bosch device struct


  // resets and then enables
  axi_iic_init();


  // IMU init
  int8_t rslt = bmi270_kria_init(&ctx, &dev);
  if (rslt != BMI2_OK){
    printf("BMI270 init failed: %d\n", rslt);
    return 1;
  }
  printf("BMI270 init result: %d\n", rslt);




  // Initialize rover
  if (rover_init() != 0) {
    printf("failed to initialize rover\n");
    return -1;
  }
  if (isr_init() != 0) {
    printf("failed to initialize rover\n");
    return -1;
  }


  // // initialize KF
  // struct bmi2_sens_axes_data acc, gyr;
  // rslt = bmi270_kria_read_ag(&dev, &acc, &gyr);
  // if (rslt == BMI2_OK){
  //   float ax = (float)(acc.x / (32768.0f / 4.0f));
  //   float ay = (float)(acc.y / (32768.0f / 4.0f));
  //   float az = (float)(acc.z / (32768.0f / 4.0f));


  //   // check for valid first reading — if all zeros, read again
  //   if (fabsf(az) < 0.1f){
  //       printf("First IMU read returned zeros, waiting for valid sample\n");
  //       bmi270_delay_us(50000, NULL);
  //       rslt = bmi270_kria_read_ag(&dev, &acc, &gyr);
  //       ax = (float)(acc.x / (32768.0f / 4.0f));
  //       ay = (float)(acc.y / (32768.0f / 4.0f));
  //       az = (float)(acc.z / (32768.0f / 4.0f));
  //   }
   
  //   printf("Kalman init with ax=%.3f ay=%.3f az=%.3f\n", ax, ay, az);
  //   kalman_init(ax, ay, -az);
  // }
  // else {
  //   printf("Initial IMU KF read failed\n");
  //   return 1;
  // }


  // keep reading until we get a valid first sample
  struct bmi2_sens_axes_data acc, gyr;
  float ax_init = 0.0f, ay_init = 0.0f, az_init = 0.0f;
  int attempts = 0;
  while (fabsf(az_init) < 0.5f && attempts < 50){
      bmi270_delay_us(20000, NULL);   // wait 20ms between attempts
      rslt = bmi270_kria_read_ag(&dev, &acc, &gyr);
      if (rslt == BMI2_OK){
          ax_init = (float)(acc.x / (32768.0f / 4.0f));
          ay_init = (float)(acc.y / (32768.0f / 4.0f));
          az_init = (float)(acc.z / (32768.0f / 4.0f));
          printf("Attempt %d: ax=%.3f ay=%.3f az=%.3f\n", attempts, ax_init, ay_init, az_init);
      }
      attempts++;
  }


  if (fabsf(az_init) < 0.5f){
      printf("Failed to get valid IMU reading after %d attempts\n", attempts);
      return 1;
  }


  printf("Kalman init with ax=%.3f ay=%.3f az=%.3f\n", ax_init, ay_init, az_init);
  kalman_init(ax_init, ay_init, -az_init);


  // initialize IMU correction system
  if (imu_correction_init(&dev) != 0){
    printf("IMU correction init failed\n");
    return 1;
  }


  // testing
  // motor_set_speed(MOTOR_MIDDLE_LEFT_WHEEL, 1000);
  //rover_move_x(10000, 128);


  printf("All systems have passed initialization!\n");


  // start moving forward at the default speed
  simba_forward(DEFAULT_MOTOR_SPEED);


  // before the loop, record start time
  struct timespec loop_start;
  clock_gettime(CLOCK_MONOTONIC, &loop_start);
  int64_t loop_start_us = (int64_t)loop_start.tv_sec * 1000000LL + loop_start.tv_nsec / 1000LL;
  int64_t test_duration_us = 15000000LL;  // 5 seconds


  // infinite loop
  while (done == false) {

    // inside the loop, check elapsed time
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t now_us = (int64_t)now.tv_sec * 1000000LL + now.tv_nsec / 1000LL;
    if (now_us - loop_start_us > test_duration_us){
        printf("Test duration reached, stopping\n");
        done = true;
        continue;
    }


    // apply imu correction
    int update_result = imu_correction_update(&dev);


    // if there is an error in the return from imu_correction_update()
    if (update_result != 0){
      if (imu_correction_is_fault()){
        printf("There is a fault, stopping\n");
        simba_stop();
        done = true;
        continue;
      }
    }


    // check terrain. If there is an issue, handle it
    if (imu_correction_is_critical() || imu_correction_is_warning()){
      imu_correction_handle_terrain(imu_correction_get_map());
    }


    // want to check motion
    int check_motion_res = imu_correction_check_motion();
    if (check_motion_res != 0){
     
      // check stalled flag
      if (imu_correction_is_stall()){
        printf("Stall has been detected, attempting to correct\n");
        // recovery is already being corrected in imu_correction_check_motion()
        simba_forward(DEFAULT_MOTOR_SPEED);
      }


      // check unintended motion flag
      if (imu_correction_is_unintended_motion()){
        printf("Unintended motion has been detected, going to stop rover\n");
        done = true;
        continue;
      }
    }


    // want to see what IMU is receiving and what the wheel encoders are reading
    int64_t frw_pos = motor_get_position(MOTOR_FRONT_RIGHT_WHEEL);
    int64_t flw_pos = motor_get_position(MOTOR_MIDDLE_LEFT_WHEEL);
    printf("ENCODERS | FRW: %ld | MLW: %ld\n", frw_pos, flw_pos);

    // apply correction and then wait 100ms
    imu_correction_apply();
    bmi270_delay_us(20000, NULL); // changed from 100ms to 50ms (can tune value later)
  }


  // want to stop the rover after while loop
  simba_stop();


  // unmap the iic and close the fd
  munmap((void*)iic_base, MAP_SIZE);
  close(fd);

}
