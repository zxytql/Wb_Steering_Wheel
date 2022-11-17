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

#include "chassis.h"

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

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

    /* Process sensor data here */

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
