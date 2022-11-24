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
#include "main.h"
#include "chassis.h"
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 32


robot_sensor_data_t robot_sensor_data;
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
  if (Helm_Chassis_Init() != 1)
  {
    /* code */
  }
  const WbNodeRef Robot_root = wb_supervisor_node_get_from_def("ROBOT");
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

  float vx = 0;
  float vy = 0;
  float wz = 0;

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
    /* 键盘控制底盘 */
    static int now_key;
    now_key = wb_keyboard_get_key();

    /*  W = 87; A = 65; S = 83; D = 68; Q = 81; E = 69;*/
    while (now_key > 0)
    {
      //printf("%d \n", now_key);
      switch (now_key)
      {
      case 87:
        vx -= 0.5;
        break;
      case 65:
        vy -= 0.5;
        break;
      case 83:
        vx += 0.5;
        break;
      case 68:
        vy += 0.5;
        break;
      case 81:
        wz += 25;
        break;
      case 69:
        wz -= 25;
        break;
        
      default:
        break;
      }
      now_key = wb_keyboard_get_key();
    }
    
    /* 获取机器人当前全局坐标 */
    robot_sensor_data.gps_value = wb_gps_get_values(gps);
    /* Process sensor data here */
    //printf("x = %f, y = %f, z = %f \n",gps_value[0],gps_value[1], gps_value[2]);

    /* 获取机器人当前航向角 */
    robot_sensor_data.imu_value = wb_inertial_unit_get_roll_pitch_yaw(imu);
    //printf("yaw = %f \n",imu_value[2]);
    //printf("x = %f, y = %f, z = %f, yaw = %f \n",robot_sensor_data.gps_value[0],robot_sensor_data.gps_value[1], robot_sensor_data.gps_value[2], robot_sensor_data.imu_value[2]);
    
    robot_sensor_data.velocity = wb_supervisor_node_get_velocity(Robot_root);
    printf(" %f, %f, %f \n",robot_sensor_data.velocity[0],robot_sensor_data.velocity[1], robot_sensor_data.velocity[2]);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    Helm_Chassis_Ctrl(vx, vy, wz, &helm_chassis, &robot_sensor_data);
    printf("dir = %f, test = %d \n", helm_chassis.dir_chassis,helm_chassis.chassis_cor);
    wb_motor_set_velocity(lf_run_motor, helm_chassis.wheel1.v_target);
    wb_motor_set_velocity(lb_run_motor, helm_chassis.wheel2.v_target);
    wb_motor_set_velocity(rb_run_motor, helm_chassis.wheel3.v_target);
    wb_motor_set_velocity(rf_run_motor, helm_chassis.wheel4.v_target);

    wb_motor_set_position(lf_dir_motor, ANGLE2RAD(helm_chassis.wheel1.angle_target));
    wb_motor_set_position(lb_dir_motor, ANGLE2RAD(helm_chassis.wheel2.angle_target));
    wb_motor_set_position(rb_dir_motor, ANGLE2RAD(helm_chassis.wheel3.angle_target));
    wb_motor_set_position(rf_dir_motor, ANGLE2RAD(helm_chassis.wheel4.angle_target));

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
