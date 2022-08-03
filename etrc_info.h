#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_

#include <list>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tuple>
#include <unistd.h>
#include "device_io.h"
#include "info_type.h"

class Luminous
{
public:
  Luminous(SensorIo *sensor_io, Camera *camera);
  void Update();
  Color color_;
  Rgb rgb_;
  Hsv hsv_;

private:
  void SetColorReference(Color c, Hsv hsv);
  void UpdateRgb();
  void UpdateHsv();
  void UpdateColor();
  SensorIo *sensor_io_;
  Camera *camera_;
  Hsv color_ref_[kColorNum];
  // clock_t before_time = 0;
};

class Odometry
{
public:
  Odometry(MotorIo *motor_io);
  void Update();
  void SaveOdometri();
  int32_t counts_r_;
  int32_t counts_l_;
  double distance = 0;
  double distance_ = 0;
  double distance_right = 0;
  double x = 0;
  double y = 0;
  double before_x = 0;
  double before_y = 0;
  double difference_x = 0;
  double difference_y = 0;
  double direction = 0;

  int32_t theta = 0;
  int curr_index = 0;
  int32_t counts_rs[100000] = {};
  int32_t counts_ls[100000] = {};
  float locate_x[100000] = {};
  float locate_y[100000] = {};
  //  unsigned long secs[100000] = {};
  float theta_[100000] = {};

private:
  MotorIo *motor_io_;
  const int8_t R = 45;
  const int8_t D = 126;
  double theta_wa = 0;
  char str[264];
};

class P_WheelsControl
{
public:
  P_WheelsControl(MotorIo *motor_io);
  void P_exec(int32_t target_power_l, int32_t target_power_r);

private:
  MotorIo *motor_io_;
  char f[256];
};

class PurePursuit
{
public:
  PurePursuit(MotorIo* motor_io, P_WheelsControl* p_wheels_control);
  void Update(double x, double y);
  double target_distance = 0;
  double target_direction = 0;
  double difference_rad = 0;
  int target[103][2] = {
      /*00*/{0, 0}, // 0
      /*01*/{100, 0},
      /*02*/{200, 0},
      /*03*/{300, 0},
      /*04*/{400, 0},
      /*05*/{500, 0},
      /*06*/{600, 0},
      /*07*/{700, 0},
      /*08*/{800, 0},
      /*09*/{900, 0},
      /*10*/{1000, 0}, // 10
      /*11*/{1070, 0},
      /*12*/{1120, 0},
      /*13*/{1170, -10},
      /*14*/{1215, -23},
      /*15*/{1310, -72},
      /*16*/{1385, -150},
      /*17*/{1441, -245},
      /*18*/{1470, -345},
      /*19*/{1472, -440},
      /*20*/{1472, -540}, // 20
      /*21*/{1472, -640},
      /*22*/{1472, -740},
      /*23*/{1472, -840},
      /*24*/{1472, -940},
      /*25*/{1468, -1050},
      /**/{1440, -1148},
      /**/{1390, -1232},
      /**/{1322, -1310},
      /**/{1240, -1372},
      /**/{1145, -1415}, // 30
      /**/{1045, -1440},
      /**/{945, -1442},
      /**/{845, -1425},
      /**/{785, -1380},
      /**/{785, -1340},
      /**/{785, -1253},
      /**/{785, -1155},
      /**/{785, -1055},
      /**/{785, -968},
      /**/{785, -894}, // 40
      /**/{785, -802},
      /**/{736, -708},
      /**/{726, -610},
      /**/{718, -520},
      /**/{785, -618},
      /**/{785, -720},
      /**/{785, -850},
      /**/{785, -930},
      /**/{785, -1010},
      /**/{785, -1102}, // 50
      /**/{785, -1208},
      /**/{785, -1300},
      /**/{785, -1378},
      /**/{795, -1425},
      /**/{895, -1442},
      /**/{995, -1442},
      /**/{1095, -1442},
      /**/{1195, -1442},
      /**/{1295, -1442},
      /**/{1395, -1442}, // 60
      /**/{1495, -1442},
      /**/{1600, -1442},
      /**/{1694, -1430},
      /**/{1781, -1381},
      /**/{1854, -1311},
      /**/{1910, -1224},
      /**/{1940, -1125},
      /**/{1942, -1025},
      /**/{1942, -925},
      /**/{1929, -814}, // 70
      /**/{1929, -720},
      /**/{1929, -625},
      /**/{1929, -530},
      /**/{1924, -433},
      /**/{1983, -330},
      /**/{2084, -330},
      /**/{2182, -330},
      /**/{2283, -380},
      /**/{2380, -474},
      /**/{2390, -538}, // 80
      /**/{2380, -588},
      /**/{2380, -638},
      /**/{2380, -688},
      /**/{2380, -738},
      /**/{2380, -788},
      /**/{2380, -838},
      /**/{2380, -888},
      /**/{2380, -938},
      /**/{2380, -988},
      /**/{2380, -1038}, // 90
      /**/{2380, -1088},
      /**/{2380, -1138},
      /**/{2380, -1188},
      /**/{2380, -1238},
      /**/{2380, -1288},
      /**/{2380, -1338},
      /**/{2380, -1438},
      /**/{2380, -1488},
      /**/{2380, -1538},
      /**/{2380, -1688}, // 100
      /**/{2380, -1638},
      /**/{2380, -1788}};


private:
  MotorIo* motor_io_; 
  Odometry *odometry_;
  P_WheelsControl *p_wheels_control_;
  double p_power_l;
  double p_power_r;
  int8_t base_p_power = 20;
  char str[256], a[256], b[256], c[256], d[256];
  int i = 0;
  double lf = 200; //先見る距離 
  double gain_kv_r = 0.057; //比例
  double gain_kv_l = 0.05; //比例
  double gain_kt_r = 5; //微分
  double gain_kt_l = 5; //微分
  double direction_odo;
};

class Localize
{
public:
  Localize(MotorIo *motor_io, P_WheelsControl *p_wheels_control);
  void Update();
  void SaveOdometry();

private:
  Odometry *odometry_;
  PurePursuit *pure_pursuit_;
  int curr_p_index = 0;
  double distance_ = 0;
  double odometry_x = 0;
  double odometry_y = 0;
  // double simu_x = 0;
  // double simu_y = 0;
  double target_distance_ = 0;
  double difference_rad_ = 0;
  int p_target_ind;
  int32_t p_counts_rs[100000] = {};
  int32_t p_counts_ls[100000] = {};
  double p_cordinate_x[100000] = {};
  double p_cordinate_y[100000] = {};
  double simulate_x[100000] = {};
  double simulate_y[100000] = {};
  int32_t target_ind[100000] = {};
  double p_target_distance[100000] = {};
  double p_difference_rad[100000] = {};
  char str[256];
  double real_distance = 0;
  // clock_t before_time = 0;
  // char str[264];
  // struct timespec now_time;
};

#endif // ETRC22_ETRC_INFO_H_
