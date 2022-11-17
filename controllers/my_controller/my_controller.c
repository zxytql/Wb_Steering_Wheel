/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>
#include <webots/keyboard.h>
#include <stdio.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
//#include "chassis.h"

/*
 * You may want to add macros here.
 */
#define TIME_STEP 32


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* 轮电机 TAG 获取 */
  WbDeviceTag lf_run_motor = wb_robot_get_device("lf_run_motor");
  WbDeviceTag lb_run_motor = wb_robot_get_device("lb_run_motor");
  WbDeviceTag rf_run_motor = wb_robot_get_device("rf_run_motor");
  WbDeviceTag rb_run_motor = wb_robot_get_device("rb_run_motor");

  /* 舵电机 TAG 获取 */
  WbDeviceTag lf_dir_motor = wb_robot_get_device("lf_dir_motor");
  WbDeviceTag lb_dir_motor = wb_robot_get_device("lb_dir_motor");
  WbDeviceTag rf_dir_motor = wb_robot_get_device("rf_dir_motor");
  WbDeviceTag rb_dir_motor = wb_robot_get_device("rb_dir_motor");

  /* 轮电机运动矢量配置为无穷 */
  wb_motor_set_position(lf_run_motor, INFINITY);
  wb_motor_set_position(lb_run_motor, INFINITY);
  wb_motor_set_position(rf_run_motor, INFINITY);
  wb_motor_set_position(rb_run_motor, INFINITY);

  /* 初始化轮电机速度 */
  wb_motor_set_velocity(lf_run_motor, 0.0);
  wb_motor_set_velocity(lb_run_motor, 0.0);
  wb_motor_set_velocity(rf_run_motor, 0.0);
  wb_motor_set_velocity(rb_run_motor, 0.0);

  /* 初始化舵电机位置 -- 弧度制 */
  wb_motor_set_position(lf_dir_motor, 0.0);
  wb_motor_set_position(lb_dir_motor, 0.0);
  wb_motor_set_position(rf_dir_motor, 0.0);
  wb_motor_set_position(rb_dir_motor, 0.0);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);
  
  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu,TIME_STEP);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1)
  {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    // static int now_key;
    // now_key = wb_keyboard_get_key();

    // /*  W = 87; A = 65; S = 83; D = 68; Q = 81; E = 69;*/
    // while (now_key > 0)
    // {
    //   //printf("%d \n", now_key);
    //   switch (now_key)
    //   {
    //   case 87:
    //     /* code */
    //     break;
      
    //   default:
    //     break;
    //   }
    //   now_key = wb_keyboard_get_key();
    // }
    
    /* 获取机器人当前全局坐标 */
    static const double *gps_value;
    gps_value = wb_gps_get_values(gps);
    /* Process sensor data here */
    //printf("x = %f, y = %f, z = %f \n",gps_value[0],gps_value[1], gps_value[2]);

    /* 获取机器人当前航向角 */
    static const double *imu_value;
    imu_value = wb_inertial_unit_get_roll_pitch_yaw(imu);
    //printf("yaw = %f \n",imu_value[2]);
    printf("x = %f, y = %f, z = %f, yaw = %f \n",gps_value[0],gps_value[1], gps_value[2], imu_value[2]);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    wb_motor_set_velocity(rf_run_motor, -2.0f);
    wb_motor_set_velocity(lf_run_motor, -2.0f);
    wb_motor_set_velocity(lb_run_motor, 2.0f);
    wb_motor_set_velocity(rb_run_motor, 2.0f);

    wb_motor_set_position(rf_dir_motor, M_PI_4);
    wb_motor_set_position(lf_dir_motor, M_PI_4 * 3);
    wb_motor_set_position(lb_dir_motor, M_PI_4);
    wb_motor_set_position(rb_dir_motor, M_PI_4 * 3);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
