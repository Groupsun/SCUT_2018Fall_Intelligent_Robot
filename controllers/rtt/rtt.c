
#include <math.h>
#include <stdio.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define TARGET_POINTS_SIZE 89
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
{-1.437500, -1.437500},
{-1.428189, -1.375698},
{-1.369177, -1.355109},
{-1.322714, -1.313307},
{-1.260378, -1.317833},
{-1.197892, -1.316525},
{-1.135397, -1.317349},
{-1.075362, -1.299969},
{-1.031473, -1.255472},
{-1.013143, -1.195720},
{-0.950997, -1.202358},
{-0.929387, -1.143713},
{-0.902709, -1.087193},
{-0.840246, -1.085039},
{-0.777754, -1.086036},
{-0.799648, -1.027497},
{-0.757645, -0.981215},
{-0.783775, -0.924440},
{-0.721588, -0.930687},
{-0.687963, -0.878003},
{-0.707237, -0.818549},
{-0.681711, -0.761500},
{-0.692128, -0.699874},
{-0.716695, -0.642405},
{-0.667496, -0.603860},
{-0.622629, -0.560349},
{-0.650144, -0.504232},
{-0.588446, -0.514212},
{-0.526768, -0.504111},
{-0.464756, -0.496314},
{-0.407727, -0.521885},
{-0.435719, -0.466004},
{-0.438756, -0.403578},
{-0.382747, -0.375844},
{-0.351592, -0.321663},
{-0.323097, -0.266036},
{-0.315070, -0.204054},
{-0.315809, -0.141558},
{-0.344835, -0.086207},
{-0.295979, -0.047228},
{-0.320176, 0.010398},
{-0.272804, -0.030371},
{-0.300961, 0.025427},
{-0.245409, 0.054067},
{-0.182909, 0.054089},
{-0.120806, 0.047049},
{-0.074172, 0.088660},
{-0.027538, 0.130271},
{-0.020586, 0.192383},
{0.030603, 0.156524},
{0.076817, 0.198602},
{0.134817, 0.221888},
{0.182998, 0.261699},
{0.170022, 0.322837},
{0.178183, 0.384801},
{0.191364, 0.445896},
{0.214890, 0.503799},
{0.270934, 0.476134},
{0.319166, 0.515882},
{0.367398, 0.555630},
{0.427380, 0.573194},
{0.474868, 0.613828},
{0.522357, 0.654461},
{0.551805, 0.709589},
{0.614165, 0.705407},
{0.657017, 0.659910},
{0.701293, 0.704022},
{0.730359, 0.759352},
{0.775468, 0.802612},
{0.820578, 0.845871},
{0.873323, 0.812343},
{0.927664, 0.781466},
{0.899739, 0.837381},
{0.857517, 0.883463},
{0.920016, 0.883284},
{0.881293, 0.932343},
{0.927560, 0.974363},
{0.939663, 1.035680},
{0.970647, 1.089959},
{1.033085, 1.087168},
{1.080325, 1.128090},
{1.127565, 1.169013},
{1.178765, 1.133168},
{1.219247, 1.180785},
{1.278840, 1.161944},
{1.310026, 1.216107},
{1.341212, 1.270271},
{1.372399, 1.324434},
{1.403585, 1.378597}
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
