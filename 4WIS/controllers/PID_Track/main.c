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
#include "webots/pen.h"
#include "fuzzy_pid.h"
#include "PID_Nav.h"
#include "webots/position_sensor.h"

/*
 * You may want to add macros here.
 */
#define TIME_STEP 32

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_robot_step(1000);
  Helm_Chassis_Init();
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

  /* 位置传感器初始化 */
  WbDeviceTag LF_Steering_Sensor = wb_robot_get_device("LF_Dir_Sensor");
  WbDeviceTag LB_Steering_Sensor = wb_robot_get_device("LB_Dir_Sensor");
  WbDeviceTag RF_Steering_Sensor = wb_robot_get_device("RF_Dir_Sensor");
  WbDeviceTag RB_Steering_Sensor = wb_robot_get_device("RB_Dir_Sensor");
  wb_position_sensor_enable(LF_Steering_Sensor, TIME_STEP);
  wb_position_sensor_enable(LB_Steering_Sensor, TIME_STEP);
  wb_position_sensor_enable(RF_Steering_Sensor, TIME_STEP);
  wb_position_sensor_enable(RB_Steering_Sensor, TIME_STEP);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP);
  
  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu,TIME_STEP);

  float vx = 0;
  float vy = 0;
  float wz = 0;
  vel_t vel;

  FILE *fp;
  //fp = fopen("./MATLAB/data_test.txt","w");
  // float kp = 0.1f;
  // float ki = 0.003f;
  // float kd = 0.21f;
  // float err = 0;
  // float err_last = 0;
  // float err_sum = 0;
  // float i_out = 0;
  // float dst_y_pos = 2000.0f;
  PID_Nav_Init(&pid_nav, &fzy_pid_x, &fzy_pid_y,&fzy_pid_w,&fzy_pid_v);
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    helm_chassis.wheel1.rad_actual = wb_position_sensor_get_value(LF_Steering_Sensor);
    helm_chassis.wheel2.rad_actual = wb_position_sensor_get_value(LB_Steering_Sensor);
    helm_chassis.wheel3.rad_actual = wb_position_sensor_get_value(RB_Steering_Sensor);
    helm_chassis.wheel4.rad_actual = wb_position_sensor_get_value(RF_Steering_Sensor);

    printf("Wheel1 angle: %f, Wheel2 angle: %f, Wheel3 angle: %f, Wheel4 angle: %f \n",RAD2ANGLE(helm_chassis.wheel1.rad_actual), 
          RAD2ANGLE(helm_chassis.wheel2.rad_actual), 
          RAD2ANGLE(helm_chassis.wheel3.rad_actual),
          RAD2ANGLE(helm_chassis.wheel4.rad_actual));
    Read_Roll_Pitch_Yaw(imu,&wb_imu_data);
    Read_GPS_Val(gps,&wb_gps_data);
    Read_SPVS_Val(&wb_spvs_data);
    //printf("Now time is: %f \n",wb_robot_get_time());
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
    //vx = fuzzypid_cal(&fzy_pid_x,Get_SPVS_X_Vel() * 1000.0f, 0, Get_GPS_X_mm());
    //vy = fuzzypid_cal(&fzy_pid_y,Get_SPVS_Y_Vel() * 1000.0f, 8.0f * 1000.0f, Get_GPS_Y_mm());
    //printf("output = %f \n",-vy);

    // PID_Nav_Point_Handler(&pid_nav,path_1);
    // vel = PID_Nav_Point_Tracker(&pid_nav, &fzy_pid_x, &fzy_pid_y, &fzy_pid_v, path_1);
    // vx = vel.vx;    
    // vy = vel.vy;
    // wz = vel.ang_w;
    //printf("nav_w = %f \n",wz);
    //fprintf(fp, "%f, %f \n",Get_GPS_X(),Get_GPS_Y());
    
    // printf("Now_index = %d \n",pid_nav.point_index);
    // printf("Now_x = %f, Now_y = %f \n",Get_GPS_X_mm(),Get_GPS_Y_mm());
    // printf("vx = %f, vy = %f\n",-vx,-vy);
    static float tgt_angle = 0.0f;
    // wz = Navigation_Angle_Ctrl(RAD2ANGLE(Get_IMU_Yaw()),tgt_angle,4.0f,4.0f); //PD低通角度环 总有稳态误差

    //wz = fuzzypid_cal(&fzy_pid_x, 0 , tgt_angle, RAD2ANGLE(Get_IMU_Yaw()));
    // printf("wz = %f, now yaw(ang) = %f \n",wz,RAD2ANGLE(Get_IMU_Yaw()));
    // fprintf(fp,"%f, %f, %f, %f \n",Get_GPS_X_mm(), Get_GPS_Y_mm(), RAD2ANGLE(Get_IMU_Yaw()), (wb_robot_get_time()-1));  //wb_robot_get_time()的单位是s 每个TIME_STEP返回一次数据
    // if(tgt_angle - RAD2ANGLE(Get_IMU_Yaw()) < 0.01f)
    // {
      // return 0;  //仿真结束
    // }
    //-------------------------
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    Helm_Chassis_Ctrl(0, 0, 90, &helm_chassis, Get_IMU_Yaw());
    //printf("dir = %f, test = %d \n", helm_chassis.dir_chassis,helm_chassis.chassis_cor);
    printf("Real vx: %f, vy: %f, w: %f \n",Get_SPVS_X_Vel(), Get_SPVS_Y_Vel(), Get_SPVS_Z_Angular_Vel());
    wb_motor_set_velocity(lf_run_motor, helm_chassis.wheel1.v_target / WHEEL_RADIUS);
    wb_motor_set_velocity(lb_run_motor, helm_chassis.wheel2.v_target / WHEEL_RADIUS);
    wb_motor_set_velocity(rb_run_motor, helm_chassis.wheel3.v_target / WHEEL_RADIUS);
    wb_motor_set_velocity(rf_run_motor, helm_chassis.wheel4.v_target / WHEEL_RADIUS);

    wb_motor_set_position(lf_dir_motor, ANGLE2RAD(helm_chassis.wheel1.angle_target));
    wb_motor_set_position(lb_dir_motor, ANGLE2RAD(helm_chassis.wheel2.angle_target));
    wb_motor_set_position(rb_dir_motor, ANGLE2RAD(helm_chassis.wheel3.angle_target));
    wb_motor_set_position(rf_dir_motor, ANGLE2RAD(helm_chassis.wheel4.angle_target));

    Helm_Evaluation(&helm_chassis);

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  //fclose(fp);
  return 0;
}
