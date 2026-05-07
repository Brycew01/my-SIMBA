// TEST MMIO GPIO CONTROL
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>


#include "rover.h"
#include "servo.h"
#include "imu_correction.h"
#include "kalman_wrapper.h"
#include "bmi270_port.h"


// globals
#define DEFAULT_MOTOR_SPEED 128
static bool done = false;


void sigint_handler(int signum) {
  (void)signum;
  printf("SIGINT caught\n");
  done = true;
}




// ------------- added some wrapper functions for simba movement ------------------------
static void simba_forward(int speed){
 
  imu_correction_set_motor_state(true);
  imu_correction_set_heading_reference();   // gets a snapshot of yaw as the reference to maintain during straight driving
  rover_forward(speed);
}


// static void simba_reverse(int speed){


//   imu_correction_set_motor_state(true);
//   imu_correction_set_heading_reference();   // gets a snapshot of yaw as the reference to maintain during straight driving
//   rover_reverse(speed);
// }


static void simba_stop(void){


  imu_correction_set_motor_state(false);
  imu_correction_clear_heading_reference();
  rover_stop();
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


  // initialize KF
  struct bmi2_sens_axes_data acc, gyr;
  rslt = bmi270_kria_read_ag(&dev, &acc, &gyr);
  if (rslt == BMI2_OK){
    float ax = (float)(acc.x / (32768.0f / 4.0f));
    float ay = (float)(acc.y / (32768.0f / 4.0f));
    float az = (float)(acc.z / (32768.0f / 4.0f));
    kalman_init(ax, ay, -az);
  }
  else {
    printf("Initial IMU KF read failed\n");
    return 1;
  }
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


  // infinite loop
  while (done == false) {


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


    // apply correction and then wait 333ms
    imu_correction_apply();
    bmi270_delay_us(333000, NULL);
  }


  // want to stop the rover after while loop
  simba_stop();


  // unmap the iic and close the fd
  munmap((void*)iic_base, MAP_SIZE);
  close(fd);


  // Close Rover
  if (rover_close() != 0) {
    printf("failed to close rover\n");
    return -1;
  }


  return 0;
}
