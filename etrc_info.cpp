#include "etrc_info.h"
#include "ctime"
// #include <time.h>

Luminous::Luminous(SensorIo *sensor_io, Camera *camera)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io), camera_(camera)
{
}

void Luminous::Update()
{
  UpdateRgb();
  UpdateHsv();
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv)
{
  color_ref_[c] = hsv;
}

void Luminous::UpdateRgb()
{
  rgb_raw_t val = sensor_io_->color_rgb_raw_;

  rgb_.r = val.r;
  rgb_.g = val.g;
  rgb_.b = val.b;
}

void Luminous::UpdateHsv()
{
  float r = static_cast<float>(rgb_.r);
  float g = static_cast<float>(rgb_.g);
  float b = static_cast<float>(rgb_.b);

  float max = r > g ? r : g;
  max = max > b ? max : b;
  float min = r < g ? r : g;
  min = min < b ? min : b;
  float c = max - min;

  float h;
  if (c == 0)
  {
    h = -1;
  }
  else if (max == r)
  {
    h = fmodf(((g - b) / c), 6);
  }
  else if (max == g)
  {
    h = ((b - r) / c) + 2;
  }
  else if (max == b)
  {
    h = ((r - g) / c) + 4;
  }
  else
  {
    h = -1;
  }

  if (h != -1)
  {
    h = 60 * h;
  }

  float s;
  if (max == 0)
  {
    s = 0;
  }
  else
  {
    s = c / max;
  }

  float v = max;
  if (v > 100)
  {
    v = 100;
  }

  hsv_.h = h;
  hsv_.s = s * 100;
  hsv_.v = v;
}

void Luminous::UpdateColor()
{
}

Odometry::Odometry(MotorIo *motor_io)
    : motor_io_(motor_io)
{
}

void Odometry::Update()
{
  counts_r_ = motor_io_->counts_r_;
  counts_l_ = motor_io_->counts_l_;

  curr_index += 1;
  counts_rs[curr_index] = counts_r_;
  counts_ls[curr_index] = counts_l_;
  locate_x[curr_index] = x;
  locate_y[curr_index] = y;

  double Ll = R * (counts_ls[curr_index] - counts_ls[curr_index - 1]) * M_PI / 180;
  double Lr = R * (counts_rs[curr_index] - counts_rs[curr_index - 1]) * M_PI / 180;

  double micro_theta = (Lr - Ll) / D;
  theta_wa += micro_theta;
  theta = counts_r_;
  double A = (Lr + Ll) / 2 * (1 - 0);
  double dx = A * cos(theta_wa + micro_theta / 2);
  double dy = (A * sin(theta_wa + micro_theta / 2));
  double dd = sqrt(dx * dx + dy * dy);

  before_x = x;
  before_y = y;

  x += dx;
  y += dy;

  difference_x = x - before_x;
  difference_y = y - before_y;
  direction = atan2(difference_y, difference_x);

  distance_ += dd;
  distance_right += A;
  theta_[curr_index] = theta_wa;

  // char str[264];
  // sprintf(str, "x: %f y: %f distance: %f distance_right: %f theta_wa:%f\n", x, y, distance_, distance_right, theta_wa);
  // syslog(LOG_NOTICE, str);

  // char str[264];
  // sprintf(str, "x: %f y: %f before_x: %f before_y: %f \n", x, y, before_x, before_y);
  // syslog(LOG_NOTICE, str);

  // char str[264];
  // sprintf(str, "diff_x: %f diff_y: %f\n", difference_x, difference_y);
  // syslog(LOG_NOTICE, str);

  // char str[264];
  // sprintf(str, "direction: %f\n", direction);
  // syslog(LOG_NOTICE, str);
}

P_WheelsControl::P_WheelsControl(MotorIo *motor_io) : motor_io_(motor_io)
{
}

void P_WheelsControl::P_exec(int32_t target_power_l, int32_t target_power_r)
{
  // int32_t curr_power_l = motor_io_->power_l_;
  // if (target_power_l > curr_power_l)
  // {
  //   curr_power_l += 1;
  // }
  // else if (target_power_l < curr_power_l)
  // {
  //   curr_power_l -= 1;
  // }

  // int32_t curr_power_r = motor_io_->power_r_;
  // if (target_power_r > curr_power_r)
  // {
  //   curr_power_r += 1;
  // }
  // else if (target_power_r < curr_power_r)
  // {
  //   curr_power_r -= 1;
  // }

  if (target_power_l == 0 && target_power_r == 0)
  {
    motor_io_->StopWheels(true);
  }
  else
  {
    motor_io_->SetWheelsPower(target_power_l, target_power_r);
  }
}

PurePursuit::PurePursuit(MotorIo *motor_io, P_WheelsControl *p_wheels_control)
    : motor_io_(motor_io), p_wheels_control_(p_wheels_control)
{
  odometry_ = new Odometry(motor_io);
}

void PurePursuit::Update(double odometry_x, double odometry_y)
{
  odometry_->Update();
  direction_odo = odometry_->direction;

  target_distance = sqrt((target[i][0] - odometry_x) * (target[i][0] - odometry_x) + (target[i][1] - odometry_y) * (target[i][1] - odometry_y));
  target_direction = atan2((target[i][1] - odometry_y), (target[i][0] - odometry_x));
  difference_rad = target_direction - direction_odo;

  while (difference_rad > 4.7) //第四象限からの変換（偏角）
  {
    difference_rad = difference_rad - 6.28;
  }
  while (difference_rad < -4.7)
  {
    difference_rad = difference_rad + 6.28;
  }

  p_power_r = gain_kv_r * target_distance + gain_kt_r * difference_rad + base_p_power;
  p_power_l = gain_kv_l * target_distance - gain_kt_l * difference_rad + base_p_power;

  int32_t ppower_l = (int)p_power_l;
  int32_t ppower_r = (int)p_power_r;
  // sprintf(a, "int_r: %d, int_l: %d, double_r: %f, double_l: %f\n", ppower_r, ppower_l, p_power_r, p_power_l);
  // syslog(LOG_NOTICE, a);
  p_wheels_control_->P_exec(ppower_l, ppower_r);
  // p_wheels_control_->P_exec(40, 40);
  sprintf(a, "i: %d, target_distance: %f, power_L: %d, power_R: %d, x: %f, y: %f\n target_direction: %f, direction_odo: %f, dif_rad: %f\n", i, target_distance, ppower_l, ppower_r, odometry_x, odometry_y, target_direction, direction_odo, difference_rad);
  syslog(LOG_NOTICE, a);

  if (target_distance < lf) //先見る距離
  {
    i = i + 1;
  }
}

Localize::Localize(MotorIo *motor_io, P_WheelsControl *p_wheels_control)
{
  odometry_ = new Odometry(motor_io);
  pure_pursuit_ = new PurePursuit(motor_io, p_wheels_control);
}

void Localize::Update()
{
  odometry_->Update();

  distance_ = odometry_->distance;
  odometry_x = odometry_->x;
  odometry_y = odometry_->y;

  // simu_x = pure_pursuit_->x;
  // simu_y = pure_pursuit_->y;
  curr_p_index = odometry_->curr_index;
  p_counts_rs[curr_p_index] = odometry_->counts_r_;
  p_counts_ls[curr_p_index] = odometry_->counts_l_;

  p_cordinate_x[curr_p_index] = odometry_x;
  p_cordinate_y[curr_p_index] = odometry_y;

  // simulate_x[curr_p_index] = simu_x;
  // simulate_y[curr_p_index] = simu_y;

  // target_ind[curr_p_index] = p_target_ind;

  real_distance = sqrt(odometry_x * odometry_x + odometry_y * odometry_y);
  // sprintf(str, "real_distance: %f\n", real_distance);

  // sprintf(str, "curr_p_index: %d\n", curr_p_index);
  // syslog(LOG_NOTICE, str);

  // sprintf(str, "x: %f y: %f\n", odometry_x, odometry_y);
  // syslog(LOG_NOTICE, str);

  pure_pursuit_->Update(odometry_x, odometry_y);
  target_distance_ = pure_pursuit_->target_distance;
  difference_rad_ = pure_pursuit_->difference_rad;
  p_target_distance[curr_p_index] = target_distance_;
  p_difference_rad[curr_p_index] = difference_rad_;
}

void Localize::SaveOdometry()
{
  FILE *fp;
  char file_name[64];
  time_t timer = time(NULL);
  struct tm* t = localtime(&timer);

  sprintf(file_name, "Pursuit_try/data/ododmetry (%d月%d日%d:%d:%d).csv", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
  fp = fopen(file_name, "w");
  // sprintf(str, "motor_l ,motar_r ,motor_l_lowpass ,motor_r_lowpass\n");
  // fprintf(fp, str);
  for (int i = 0; i < curr_p_index;  i++) {
    sprintf(str, "%d, %d, %f, %f, \n", p_counts_ls[i], p_counts_rs[i], p_cordinate_x[i], p_cordinate_y[i]);
    fprintf(fp, str);
  }

  fclose(fp);


  // FILE *fp = fopen("Pursuit.csv", "w");
  // char str[256];

  // for (int i=0; i< curr_p_index; i++) {
  // sprintf(str, "%f, %f\n", simulate_x[i], simulate_y[i]);
  // // sprintf(str, "%d, %d\n", p_counts_rs[i], p_counts_ls[i]);
  // fprintf(fp, str);
  // }

  // for (int i = 0; i < curr_p_index; i++)
  // {
  //   sprintf(str, "%f, %f\n", p_target_distance[i], p_difference_rad[i]);
  //   fprintf(fp, str);
  // }

  // for (int i = 0; i < curr_p_index; i++)
  // {
  //   sprintf(str, "%d, %d, %f, %f, \n", p_counts_ls[i], p_counts_rs[i], p_cordinate_x[i], p_cordinate_y[i]);
  //   // sprintf(str, "%d, %d\n", p_counts_rs[i], p_counts_ls[i]);
  //   fprintf(fp, str);
  // }

  // sprintf(str, "%d, %d, %d\n", p_counts_rs[curr_index], counts_ls[curr_index], curr_index);
  // // sprintf(str, "%d\n", curr_p_index);
  // syslog(LOG_NOTICE, str);

  // for (int i=0; i<curr_index; i++) {
  //   sprintf(str, "%f, %f\n", locate_x[i], locate_y[i]);
  //   fprintf(fp, str);
  // }

  // for (int i=0; i< curr_index; i++) {
  //   sprintf(str, "%d, %d\n", counts_rs[i], counts_ls[i]);
  //   fprintf(fp, str);
  // }

  // sprintf(str, "%d\n", curr_index);
  // syslog(LOG_NOTICE, str);

  //  for (int i=0; i<curr_index; i++) {
  //    sprintf(str, "%u\n", secs[i]);
  //    fprintf(fp, str);
  //  }

  // for (int i=0; i<curr_index; i++) {
  //   sprintf(str, "%f\n", theta_[i]*180/M_PI);
  //   fprintf(fp, str);
  // }
  // fclose(fp);
}
