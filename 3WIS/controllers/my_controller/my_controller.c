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
#include "chassis.h"
#include "sensor.h"
#include <stdio.h>
#include <webots/position_sensor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 16

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

    /* 轮电机 TAG 获取 */
  WbDeviceTag F_Driving_Motor = wb_robot_get_device("F_Driving_Motor");
  WbDeviceTag L_Driving_Motor = wb_robot_get_device("L_Driving_Motor");
  WbDeviceTag R_Driving_Motor = wb_robot_get_device("R_Driving_Motor");

  /* 舵电机 TAG 获取 */
  WbDeviceTag F_Steering_Motor = wb_robot_get_device("F_Steering_Motor");
  WbDeviceTag L_Steering_Motor = wb_robot_get_device("L_Steering_Motor");
  WbDeviceTag R_Steering_Motor = wb_robot_get_device("R_Steering_Motor");

  /* 轮电机运动矢量配置为无穷 */
  wb_motor_set_position(F_Driving_Motor, INFINITY);
  wb_motor_set_position(L_Driving_Motor, INFINITY);
  wb_motor_set_position(R_Driving_Motor, INFINITY);

  /* 初始化轮电机速度 */
  wb_motor_set_velocity(F_Driving_Motor, 0.0);
  wb_motor_set_velocity(L_Driving_Motor, 0.0);
  wb_motor_set_velocity(R_Driving_Motor, 0.0);

  /* 初始化舵电机位置 -- 弧度制 */
  wb_motor_set_position(F_Steering_Motor, 0.0);
  wb_motor_set_position(L_Steering_Motor, 0.0);
  wb_motor_set_position(R_Steering_Motor, 0.0);

  /* 位置传感器初始化 */
  WbDeviceTag F_Steering_Sensor = wb_robot_get_device("F_Steering_Sensor");
  WbDeviceTag L_Steering_Sensor = wb_robot_get_device("L_Steering_Sensor");
  WbDeviceTag R_Steering_Sensor = wb_robot_get_device("R_Steering_Sensor");
  wb_position_sensor_enable(F_Steering_Sensor, TIME_STEP);
  wb_position_sensor_enable(L_Steering_Sensor, TIME_STEP);
  wb_position_sensor_enable(R_Steering_Sensor, TIME_STEP);


  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);
  
  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu,TIME_STEP);

  while (!Helm_Chassis_Init()); //等待底盘初始化

  while (wb_robot_step(TIME_STEP) != -1) {

    Read_SPVS_Val(&wb_spvs_data);
    Read_Roll_Pitch_Yaw(imu, &wb_imu_data);

    helm_chassis.wheel1.rad_actual = wb_position_sensor_get_value(F_Steering_Sensor);
    helm_chassis.wheel2.rad_actual = wb_position_sensor_get_value(L_Steering_Sensor);
    helm_chassis.wheel3.rad_actual = wb_position_sensor_get_value(R_Steering_Sensor);
    helm_chassis.chassis_ct.ct_vx = Get_SPVS_X_Vel();
    helm_chassis.chassis_ct.ct_vy = Get_SPVS_Y_Vel();
    helm_chassis.chassis_ct.ct_wz = Get_SPVS_Z_Angular_Vel();
    printf("Wheel1 angle: %f, Wheel2 angle: %f, Wheel3 angle: %f \n",RAD2ANGLE(helm_chassis.wheel1.rad_actual), 
          RAD2ANGLE(helm_chassis.wheel2.rad_actual), 
          RAD2ANGLE(helm_chassis.wheel3.rad_actual));

    Helm_Chassis_Ctrl(0, 0, 45, &helm_chassis, Get_IMU_Yaw()); // wz unit is °/s

    wb_motor_set_velocity(F_Driving_Motor, helm_chassis.wheel1.v_target / WHEEL_RADIUS);
    wb_motor_set_velocity(L_Driving_Motor, helm_chassis.wheel2.v_target / WHEEL_RADIUS);
    wb_motor_set_velocity(R_Driving_Motor, helm_chassis.wheel3.v_target / WHEEL_RADIUS);

    wb_motor_set_position(F_Steering_Motor, ANGLE2RAD(helm_chassis.wheel1.angle_target));
    wb_motor_set_position(L_Steering_Motor, ANGLE2RAD(helm_chassis.wheel2.angle_target));
    wb_motor_set_position(R_Steering_Motor, ANGLE2RAD(helm_chassis.wheel3.angle_target));

    printf("Real Vx: %f m/s, Vy: %f m/s, W: %f \n",helm_chassis.chassis_ct.ct_vx, helm_chassis.chassis_ct.ct_vy, helm_chassis.chassis_ct.ct_wz);
    
    Helm_Evaluation(&helm_chassis);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
