
#include <math.h>
#include <stdio.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define TARGET_POINTS_SIZE 46
#define DISTANCE_TOLERANCE 0.01
#define ANGLE_TOLERANCE 0.05
#define MAX_SPEED 5.0

enum XYZAComponents { X, Y, Z, ALPHA };
enum Sides { LEFT, RIGHT };

typedef struct _Vector {
  double u;
  double v;
} Vector;

static WbDeviceTag left_motor;
static WbDeviceTag right_motor;
static WbDeviceTag gps;
static WbDeviceTag compass;

static Vector targets[TARGET_POINTS_SIZE] ={
  {-1.312500, -1.437500},
  {-1.187500, -1.437500},
  {-1.062500, -1.437500},
  {-1.062500, -1.312500},
  {-1.062500, -1.187500},
  {-1.062500, -1.062500},
  {-0.937500, -1.062500},
  {-0.812500, -1.062500},
  {-0.687500, -1.062500},
  {-0.687500, -0.937500},
  {-0.562500, -0.937500},
  {-0.562500, -0.812500},
  {-0.562500, -0.687500},
  {-0.562500, -0.562500},
  {-0.437500, -0.562500},
  {-0.312500, -0.562500},
  {-0.312500, -0.437500},
  {-0.312500, -0.312500},
  {-0.312500, -0.187500},
  {-0.312500, -0.062500},
  {-0.187500, -0.062500},
  {-0.062500, -0.062500},
  {0.062500, -0.062500},
  {0.187500, -0.062500},
  {0.312500, -0.062500},
  {0.437500, -0.062500},
  {0.562500, -0.062500},
  {0.687500, -0.062500},
  {0.687500, 0.062500},
  {0.687500, 0.187500},
  {0.812500, 0.187500},
  {0.937500, 0.187500},
  {0.937500, 0.312500},
  {0.937500, 0.437500},
  {0.937500, 0.562500},
  {0.937500, 0.687500},
  {0.937500, 0.812500},
  {0.937500, 0.937500},
  {0.937500, 1.062500},
  {1.062500, 1.062500},
  {1.187500, 1.062500},
  {1.312500, 1.062500},
  {1.437500, 1.062500},
  {1.437500, 1.187500},
  {1.437500, 1.312500},
  {1.437500, 1.437500}
};

static int current_target_index = 0;


static double modulus_double(double a, double m) {
  int div_i = (int)(a / m);
  double div_d = (double)div_i;
  double r = a - div_d * m;
  if (r < 0.0)
    r += m;
  return r;
}

static void robot_set_speed(double left, double right) {
  wb_motor_set_velocity(left_motor, left);
  wb_motor_set_velocity(right_motor, right);
}

static double norm(const Vector *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

// v = v/||v||
static void normalize(Vector *v) {
  double n = norm(v);
  v->u /= n;
  v->v /= n;
}


static void minus(Vector *v, const Vector *v1, const Vector *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}


static double angle(const Vector *v1, const Vector *v2) {
  return modulus_double(atan2(v2->v, v2->u) - atan2(v1->v, v1->u), 2.0 * M_PI);
}

static void run_autopilot() {
  double speeds[2] = {0.0, 0.0};

  const double *pos3D = wb_gps_get_values(gps);
  const double *north3D = wb_compass_get_values(compass);

  Vector pos = {pos3D[X], pos3D[Z]};
  Vector north = {north3D[X], north3D[Z]};
  Vector front = {-north.u, north.v};

  Vector dir;
  minus(&dir, &(targets[current_target_index]), &pos);
  double distance = norm(&dir);
  normalize(&dir);

  double beta = angle(&front, &dir) - M_PI;

  if (distance < DISTANCE_TOLERANCE) {
    printf("target %d reached\n", current_target_index + 1);
    current_target_index++;
  }
  else {
    if (beta > ANGLE_TOLERANCE) {
      speeds[LEFT] = MAX_SPEED * beta / M_PI ;
      speeds[RIGHT] = -MAX_SPEED * beta /M_PI;
    } else if (beta < -ANGLE_TOLERANCE) {
      speeds[LEFT] = -MAX_SPEED;
      speeds[RIGHT] = MAX_SPEED;
    }
    else {
      speeds[LEFT] = MAX_SPEED - M_PI + beta;
      speeds[RIGHT] = MAX_SPEED - M_PI - beta;

    }
  }

  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
}

int main(int argc, char *argv[]) {
  wb_robot_init();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);


  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pos3D = wb_gps_get_values(gps);
    //printf("position: {%f, %f}\n", pos3D[X], pos3D[Z]);
    if(current_target_index >= TARGET_POINTS_SIZE){
      robot_set_speed(0, 0);
      printf("approaching destination...... \n");
    }else{
      run_autopilot();
    }
  }

  wb_robot_cleanup();

  return 0;
}
