/*
 * Copyright 1996-2018 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  An example of use of the distance sensors
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <stdio.h>
#include <math.h>


#define NB_SENSORS 8
#define TIME_STEP 64
#define RANGE 1024
#define PI 3.14


int main(void) {
  int i, j;
  WbDeviceTag ps[NB_SENSORS];
  char name[] = "ds0";
  double speed[2];
  double sensor_value[NB_SENSORS];
  double matrix[2][NB_SENSORS] = {{11, 12, 8, -2, -3, -5, -7, -9}, {-9, -8, -5, -1, -2, 6, 12, 11}};

  wb_robot_init();

  /* get handlers to and enable the distance sensor devices */
  for (i = 0; i < NB_SENSORS; i++) {
    ps[i] = wb_robot_get_device(name); /* get a handler to the sensor */
    /* Perform distance measurements every TIME_STEP millisecond. */
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    /* Increase the device name to "ir1", "ir2", etc. */
    name[2]++;
  }


  /* get a handler to the motors and set target position to infinity (speed control). */
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag arrow = wb_robot_get_device("arrow");
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);



  /* control loop */
  while (wb_robot_step(TIME_STEP) != -1) {

    double x = 0;
    double y = 0;
    
    for (i = 0; i < NB_SENSORS; i++){
      sensor_value[i] = wb_distance_sensor_get_value(ps[i]);
      //printf("%f \n", sensor_value[i]/RANGE);
      x += sensor_value[i]/RANGE * cos(0.596 * PI - 0.25 * (i+1)* PI);
      y += sensor_value[i]/RANGE * sin(0.596 * PI - 0.25 * (i+1) * PI);
    }
    
    x= x;
    y = -y;
   
    
    for (i = 0; i < 2; i++) {
      speed[i] = 0;
      for (j = 0; j < NB_SENSORS; j++) {
        /*
         * We need to recenter the value of the sensor to be able to get
         * negative values too. This will allow the wheels to go
         * backward too.
         */
        speed[i] += matrix[i][j] * (1 - (sensor_value[j] / RANGE));
      }
    }

   

    const double *north = wb_compass_get_values(compass);
    //double angle = atan2(north[0], north[2]);
    double angle = atan2(north[0] - 1.5 * x, north[2] - 1.5 * y);
    printf("angle %f\n", angle);
    printf("%f \n ", angle);
    // printf("speed[0] %f\n", speed[0]);
    // printf("speed[1] %f\n", speed[1]);
    printf("north[0] %f\n", north[0]);
    printf("north[2] %f\n", north[2]);
    printf("x %f \n", x);
    printf("y %f \n", y);
    wb_motor_set_position(arrow, angle);
    
     /* Set the motor speeds */
     
    
    if(angle > 0.0872665 && angle > 0){
      printf("tunring............ \n");
      if(angle > 0){
         wb_motor_set_velocity(left_motor,  0.5);
         wb_motor_set_velocity(right_motor, -.05);
      }else{
         wb_motor_set_velocity(left_motor,  -0.5);
         wb_motor_set_velocity(right_motor, 0.5);
      }
      continue;
    }
    
    if(angle > -3.0543263 && angle < 0){
      printf("tunring............ \n");
      if(angle > 0){
         wb_motor_set_velocity(left_motor,  0.5);
         wb_motor_set_velocity(right_motor, -0.5);
      }else{
         wb_motor_set_velocity(left_motor,  -0.5);
         wb_motor_set_velocity(right_motor, 0.5);
      }
      continue;
    }
    
     printf("go ahead ....... \n");
     wb_motor_set_velocity(left_motor,  2);
     wb_motor_set_velocity(right_motor, 2);

   
    
    
    
    
    printf("left motor %f \n", fabs(-PI/2 - angle));
    printf("right motor %f \n", fabs(-1.5 * PI - angle));


  }

  wb_robot_cleanup();

  return 0;
}
