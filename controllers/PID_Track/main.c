/*
 * File:          main.c
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
#include "bsp_imu.h"
#include "bsp_gps.h"
#include "bsp_spvs.h"
#include <stdio.h>

#include "fuzzy_pid.h"
#include "PID_Nav.h"
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_robot_step(1000);
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
  vel_t vel;
  // float kp = 0.1f;
  // float ki = 0.003f;
  // float kd = 0.21f;
  // float err = 0;
  // float err_last = 0;
  // float err_sum = 0;
  // float i_out = 0;
  // float dst_y_pos = 2000.0f;
  PID_Nav_Init(&pid_nav, &fzy_pid_x, &fzy_pid_y);
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    Read_Roll_Pitch_Yaw(imu,&wb_imu_data);
    Read_GPS_Val(gps,&wb_gps_data);
    Read_SPVS_Val(&wb_spvs_data);

    /* Process sensor data here */
    //printf("X_Pos = %f, Y_Pos = %f \n",Get_GPS_X(),Get_GPS_Y());
    // printf("Yaw = %f \n",Get_IMU_Yaw());
    //printf("X_Vel = %f, Y_Vel = %f, W_AngVel = %f \n",Get_SPVS_X_Vel(), Get_SPVS_Y_Vel(), Get_SPVS_Z_Angular_Vel());

    //PID位置控制--------------
    // err = dst_y_pos - Get_GPS_Y_ms();
    // if (abs(err) < 100.0f)
    // {
    //   i_out += ki * err;
    // }
    
    // vy = kp * err + i_out + kd * (err - err_last);
    // printf("err = %f, output = %f \n",err,-vy);
    // err_last = err;
    //------------------------

    //模糊PID控制 单位mm
    // vx = fuzzypid_cal(&track_x_pid,Get_SPVS_X_Vel() * 1000.0f, 15.0f * 1000.0f, Get_GPS_X_ms());
    // vy = fuzzypid_cal(&track_y_pid,Get_SPVS_Y_Vel() * 1000.0f, 15.0f * 1000.0f, Get_GPS_Y_ms());
    // printf("output = %f \n",-vy);
    vel = PID_Nav_Func(&pid_nav, &fzy_pid_x, &fzy_pid_y, path_1);
    printf("Now_index = %d \n",pid_nav.point_index);
    printf("Now_x = %f, Now_y = %f \n",Get_GPS_X_mm(),Get_GPS_Y_mm());
    printf("vx = %f, vy = %f\n",-vx,-vy);

    //TO-DO
    //2023.5.6 对于单个点的跟踪没有问题 目前没有做速度规划（即max_plan_v） 导致在跟踪离散点时顿挫感非常严重 速度规划应该是在规划速度上做速度的增减量控制 
    
    vx = vel.vx;    
    vy = vel.vy;
    //-------------------------

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    Helm_Chassis_Ctrl(-vx, -vy, wz, &helm_chassis, Get_IMU_Yaw());
    //printf("dir = %f, test = %d \n", helm_chassis.dir_chassis,helm_chassis.chassis_cor);
    
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
