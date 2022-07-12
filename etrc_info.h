#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_


#include <list>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tuple>

#include "device_io.h"
#include "info_type.h"

const int kCourseParamNum = 23;

class Luminous {
 public:
  Luminous(SensorIo* sensor_io, Camera* camera);
  void Update();
  Color color_;
  Rgb rgb_;
  Hsv hsv_;

 private:
  void SetColorReference(Color c, Hsv hsv);
  void UpdateRgb();
  void UpdateHsv();
  void UpdateColor();
  SensorIo* sensor_io_;
  Camera* camera_;
  Hsv color_ref_[kColorNum];
  // clock_t before_time = 0;
};

class Odometry {
  public:
   Odometry(MotorIo* motor_io);
   void Update();
   void SaveOdometri();
   double distance = 0;
   double distance_ = 0;
   double distance_right = 0;
   double x = 0;
   double y = 0;
   int32_t theta = 0;
   int curr_index = 0;
   int32_t counts_rs[100000] = {};
   int32_t counts_ls[100000] = {};
   float locate_x[100000] = {};
   float locate_y[100000] = {};
  //  unsigned long secs[100000] = {};
   float theta_[100000] = {};

  private:
   MotorIo* motor_io_;
   const int8_t R = 45;
   const int8_t D = 126;
   double theta_wa = 0;
   char str[264];
};

class CubicSpline {
  // public:
  //  CubicSpline();
  //  void setCourseParam();
  //  double CalcEndpoint(const std::list<double> y);
  //  double Calc(double t);
  //  double accl;

  // private:
  //  std::list<double> a_;
  //  std::list<double> b_;
  //  std::list<double> c_;
  //  std::list<double> d_;
  //  std::list<double> w_;
};

class PurePursuit {
  public:
   PurePursuit();
   double x, y, yaw;
   void Update(double x, double y);

  private:
   double calc_distance(double point_x, double point_y);
   void readTargetCourseCoordinate();
   std::tuple<int, double> pursuit_control(int ind);
   std::tuple<int, double> search_target_index();

   int pre_point_index;
   const double lf = 0;

   const float course_x[kCourseParamNum] = {};
   const float course_y[kCourseParamNum] = {};
   
  //  CubicSpline* cubic_spline_;
};

class Localize {
 public:
  Localize(MotorIo* motor_io);
  void Update();
  double distance_ = 0;
  double odometry_x = 0;
  double odometry_y = 0;

 private:
  Odometry* odometry_;
  PurePursuit* pure_pursuit_;
  // clock_t before_time = 0;
  //char str[264];
  // struct timespec now_time;
};

#endif  // ETRC22_ETRC_INFO_H_
