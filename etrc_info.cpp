#include "etrc_info.h"

Luminous::Luminous(SensorIo* sensor_io, Camera* camera)
    : color_(kInvalidColor), hsv_({0, 0, 0}), sensor_io_(sensor_io), camera_(camera) {
}

void Luminous::Update() {
  UpdateRgb();
  UpdateHsv();
  UpdateColor();
}

void Luminous::SetColorReference(Color c, Hsv hsv) {
  color_ref_[c] = hsv;
}

void Luminous::UpdateRgb() {
  rgb_raw_t val = sensor_io_->color_rgb_raw_;
  
  rgb_.r = val.r;
  rgb_.g = val.g;
  rgb_.b = val.b;
}

void Luminous::UpdateHsv() {
  float r = static_cast<float>(rgb_.r);
  float g = static_cast<float>(rgb_.g);
  float b = static_cast<float>(rgb_.b);

  float max = r > g ? r : g;
  max = max > b ? max : b;
  float min = r < g ? r : g;
  min = min < b ? min : b;
  float c = max - min;

  float h;
  if (c == 0) {
    h = -1;
  } else if (max == r) {
    h = fmodf(((g - b) / c), 6);
  } else if (max == g) {
    h = ((b - r) / c) + 2;
  } else if (max == b) {
    h = ((r - g) / c) + 4;
  } else {
    h = -1;
  }

  if (h != -1) {
    h = 60 * h;
  }

  float s;
  if (max == 0) {
    s = 0;
  } else {
    s = c / max;
  }

  float v = max;
  if (v > 100) {
    v = 100;
  }

  hsv_.h = h;
  hsv_.s = s * 100;
  hsv_.v = v;
}

void Luminous::UpdateColor() {
}

Odometry::Odometry(MotorIo* motor_io)
  : motor_io_(motor_io) {
}

void Odometry::Update(){
  // unsigned long sec;
  // clock_gettime(CLOCK_REALTIME, &now_time);
  // sec = now_time.tv_sec;
  // secs[curr_index] = sec;
  // curr_index += 1;

  // unsigned long nsec;
  // clock_gettime(CLOCK_REALTIME, &now_time);
  // nsec = now_time.tv_nsec;
  // secs[curr_index] = nsec;
  // curr_index += 1;
  // nsec = now_time.tv_nsec;
  // clock_t now = clock();
  // // double a = (static_cast<double>(now-before_time))/CLOCKS_PER_SEC;
  // // double keep += a;  
  // sprintf(str, "time: %f sum: %f\n", (static_cast<double>(now-before_time))/CLOCKS_PER_SEC);
  // syslog(LOG_NOTICE, str);
  // before_time = now;
  
  counts_r_ = motor_io_->counts_r_;
  counts_l_ = motor_io_->counts_l_;

  // if(counts_l_ > 50 && counts_r_ > 50) {
  //   counts_l_=0;
  //   counts_r_=0;
  // }

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
  double dy =  (A * sin(theta_wa + micro_theta / 2));
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

  // sprintf(str, "%d, %d, %d\n", counts_rs[curr_index], counts_ls[curr_index], curr_index);
  // syslog(LOG_NOTICE, str);

  // theta_[curr_index] = micro_theta;

  // char str[264];
  // sprintf(str, "x: %f y: %f distance: %f distance_right: %f theta_wa:%f\n", x, y, distance_, distance_right, theta_wa);
  // syslog(LOG_NOTICE, str);

  // char str[264];
  // sprintf(str, "theta: %f\n", micro_theta*180/M_PI);
  // syslog(LOG_NOTICE, str);

  // char str[264];
  // sprintf(str, "theta: %f\n", theta_wa*180/M_PI);
  // syslog(LOG_NOTICE, str);  
}


// CubicSpline::CubicSpline() {
//   setCourseParam();
// }

// void CubicSpline::setCourseParam() {
//   int data = kCourseParamNum - 1;
//   for (int i = 0; i <= data; i++) {
//     a_.push_back(y[i]);
//   }
//   for (int i = 0; i < data; i++) {
//     if (i == 0) {
//       c_.push_back(0.0);
//     } else if (i == data) {
//       c_.push_back(0.0);
//     } else {
//       c_.push_back(3.0 * (a_[i-1] - 2.0 * a_[i] + a_[i+1]));
//     }
//   }
//   for (int i = 0; i < data; i++) {
//     if (i == 0) {
//       w_.push_back(0.0);
//     } else {
//       double tmp = 4.0 - w_[i-1];
//       c_[i] = (c_[i] - c_[i-1]) / tmp;
//       w_.push_back(1.0 / tmp);
//     }
//   }
//   for (int i = (data-1); i > 0; i--){
//       c_[i] = c_[i] - c_[i+1] * w_[i];
//   }
//   for (int i = 0; i <= data; i++) {
//     if (i == data) {
//       d_.push_back(0.0);
//       b_.push_back(0.0);
//     } else {
//       d_.push_back((c_[i+1] - c_[i]) / 3.0);
//       b_.push_back(a_[i+1] - a_[i] - c_[i] - d_[i]);
//     }
//   }
// }
// double CubicSpline::CalcEndpoint(const std::list<double> y){
//     int dt = y.size();
//     double dy = b_[j] + (c_[j] + d_[j] * dt) * dt;
//     return dy * dy;
//     return 0;
// }
// double CubicSpline::Calc(double t) {
//   int j = int(floor(t));
//   if (j < 0) {
//     j = 0;
//   } else if(j >= a_.size()) {
//     j = a_.size() - 1;
//   }
//   double dt = t - j;
//   double result = a_[j] + (b_[j] + (c_[j] + d_[j] * dt) * dt) * dt;
//   accl = 2 * c_[j] + 6 * d_[j] * dt;
//   return result;
// }

P_WheelsControl::P_WheelsControl(MotorIo* motor_io) : motor_io_(motor_io) {
}

void P_WheelsControl::P_exec(int32_t target_power_l, int32_t target_power_r) {
  // int32_t curr_power_l = motor_io_->power_l_;
  // if (target_power_l > curr_power_l) {
  //   curr_power_l += 1;
  // } else if (target_power_l < curr_power_l) {
  //   curr_power_l -= 1;
  // }

  // int32_t curr_power_r = motor_io_->power_r_;
  // if (target_power_r > curr_power_r) {
  //   curr_power_r += 1;


  // } else if (target_power_r < curr_power_r) {
  //   curr_power_r -= 1;
  // }


  if (target_power_l == 0 && target_power_r == 0) {
    motor_io_->StopWheels(true);
  } else {
    motor_io_->SetWheelsPower(target_power_l, target_power_r);
  }
}

PurePursuit::PurePursuit(MotorIo *motor_io, P_WheelsControl* p_wheels_control)
  : motor_io_(motor_io), p_wheels_control_(p_wheels_control), x(0), y(0), yaw(M_PI) {
  // cubic_spline_ = new CubicSpline();
  // readTargetCourseCoordinate();
  odometry_ = new Odometry(motor_io);
  pre_point_index = INT_MAX;
}

 
//  void PurePursuit::read_trajectry_file_x(){
//   std::string str_buf;
//   std::string str_conma_buf;

//   std::string ifs_csv_file_path_x  = "course_x.csv";
//   std::ifstream ifs_csv_file_x(ifs_csv_file_path_x);
//   int i = 0;
//   while (getline(ifs_csv_file_x, str_buf)) { 
//     std::istringstream i_stream(str_buf);
//     while (getline(i_stream, str_conma_buf, ',')) {
//     double pre = stod(str_conma_buf);
//     // course_x[i] = pre;
//     i++;
//     }
//   }
//  }

// void PurePursuit::readTargetCourseCoordinate() {
//   // for (int i=0; i<size; i++) {
//   //   course_x[i] = ;
//   //   course_y[i] = ;
//   // }
// }

double PurePursuit::calc_distance(double point_x, double point_y) {
  double dx = x - point_x;
  double dy = y - point_y;

  return hypot(dx, dy);
}


std::tuple<int, double> PurePursuit::search_target_index() {
  // int ind;
  if (pre_point_index == INT_MAX) {
    std::list<int> d;

    for (int i = 0; i < kCourseParamNum; i++) {
      double dx = x - course_x[i];
      double dy = y - course_y[i];

      // sprintf(str, "x: %f, y: %f \n", x, y);
      // syslog(LOG_NOTICE, str);

      d.push_back(hypot(dx, dy));
    }
    std::list<int>::iterator minIt = std::min_element(d.begin(), d.end());
    ind = std::distance(d.begin(), minIt);
    pre_point_index = ind;
    // sprintf(str, "ind: %d \n",ind);
    // syslog(LOG_NOTICE, str);
  } else {
    ind = pre_point_index;
    double distance = calc_distance(course_x[ind],course_y[ind]);

    while (true) {
      double next_distance = calc_distance(course_x[ind+1], course_y[ind+1]);
      if (distance < next_distance) break;
      if (ind + 1 < kCourseParamNum) {
        ind++;

      // sprintf(b, "b: %d \n",1);
      // syslog(LOG_NOTICE, b);

      }

      distance = next_distance;
    }

    pre_point_index = ind;
  }

  while (lf > calc_distance(course_x[ind], course_y[ind])) {
    if (ind > kCourseParamNum) break;
    ind += 1;

      //sprintf(c, "c: %d \n",2);

      //syslog(LOG_NOTICE, c);
  }

  return std::forward_as_tuple(ind, lf);
}

std::tuple<int, double> PurePursuit::pursuit_control(int pind) {
  int target_ind;
  double lf;
  
  std::tie(target_ind, lf) = search_target_index();

  if (pind >= target_ind) {
    target_ind = pind;
  }

  double tx,ty;
  if (target_ind < kCourseParamNum) {
      tx = course_x[target_ind];
      ty = course_y[target_ind];

  } else {
      tx = course_x[kCourseParamNum-1];
      ty = course_y[kCourseParamNum-1];
      target_ind = kCourseParamNum-1;
  }

  double alpha = atan2(ty - y, tx - x);

    sprintf(d, "x: %f, y: %f\n", x, y);
    syslog(LOG_NOTICE, d);

  return std::forward_as_tuple(target_ind, alpha);
}


void PurePursuit::Update(double odometry_x, double odometry_y) {
  // double lf;
  // double delta;
  // int target_ind;
  odometry_->Update();
  direction_odo = odometry_->direction;
  
  if (pre_point_index == INT_MAX) {
  std::tie(target_ind, p_lf) = search_target_index();
  }


  // std::tie(target_ind, lf) = search_target_index();

   //sprintf(a, "target_ind: %d\n", target_ind);
   //syslog(LOG_NOTICE, a);

  std::tie(target_ind, delta) = pursuit_control(target_ind);
     target_distance = calc_distance(course_x[target_ind], course_y[target_ind]);
     target_direction = delta;
     //double L = calc_distance(177, 156);
    //turning_radius = L / (2*sin(delta));
    //  omega = base_p_power * 2 * sin(delta) / L;
    //  p_lr = (turning_radius - p_d/2) * delta;
    //  p_ll = (turning_radius + p_d/2) * delta;
      // omega = omega * para;
      // p_power_r = base_p_power + omega;
      // p_power_l = base_p_power - omega;

  
  
  // double theta = M_PI;
  // // double theta = 0; 
  // double L = 100;
  // if (theta == 0 || theta == M_PI) turning_radius = 0; 
  // else turning_radius = L / (2 * sin(theta));
  // p_lr = (turning_radius - p_d/2) * theta;
  // p_ll = (turning_radius + p_d/2) * theta;

  // p_power_r = 70;
  // p_power_l = 70;

  // p_power_r = base_p_power * p_lr/(p_lr + p_ll);
  // p_power_l = base_p_power * p_ll/(p_lr + p_ll);


  // sprintf(str, "target_ind: %d\n", target_ind);

  int32_t ppower_l =  (int)p_power_l; 
  int32_t ppower_r =  (int)p_power_r; 

   sprintf(a, "int_r: %d, int_l: %d, double_r: %f, double_l: %f, omega: %f, target_ind: %d\n", ppower_r, ppower_l, p_power_r, p_power_l, delta, target_ind);
   syslog(LOG_NOTICE, a);

  p_wheels_control_->P_exec(ppower_l, ppower_r);

  // p_wheels_control_->P_exec(-300, 128);

  //x += v * cos(delta); 
  //y += v * sin(delta); 

  x = odometry_x;//更新
  y = odometry_y;//更新　一番最後に
  //方位角から回転角
  //回転角からモータパワー
}

Localize::Localize(MotorIo* motor_io, P_WheelsControl* p_wheels_control) {
  odometry_ = new Odometry(motor_io);
  pure_pursuit_ = new PurePursuit(motor_io, p_wheels_control);
}

void Localize::Update() {
  odometry_->Update();
  distance_ = odometry_->distance;
  odometry_x = odometry_->x;
  odometry_y = odometry_->y;

  simu_x = pure_pursuit_ -> x;
  simu_y = pure_pursuit_ -> y;

  // p_target_ind = pure_pursuit_-> target_ind;

  curr_p_index = odometry_->curr_index;
  p_counts_rs[curr_p_index] = odometry_->counts_r_;
  p_counts_ls[curr_p_index] = odometry_->counts_l_;

  p_cordinate_x[curr_p_index] = odometry_x;
  p_cordinate_y[curr_p_index] = odometry_y;

  simulate_x[curr_p_index] = simu_x;
  simulate_y[curr_p_index] = simu_y;

  // target_ind[curr_p_index] = p_target_ind;



  real_distance = sqrt(odometry_x * odometry_x + odometry_y * odometry_y);
  // sprintf(str, "real_distance: %f\n", real_distance);

  // sprintf(str, "curr_p_index: %d\n", curr_p_index);
  // syslog(LOG_NOTICE, str); 

  pure_pursuit_->Update(odometry_x, odometry_y);
}

void Localize::SaveOdometry() {
  FILE *fp;
  // char file_name[64];
  // time_t timer = time(NULL);
  // struct tm* t = localtime(&timer);

  // sprintf(file_name, "Pure_try/data/ododmetry (%d月%d日%d:%d:%d).csv", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
  // fp = fopen(file_name, "w");
  // // sprintf(str, "motor_l ,motar_r ,motor_l_lowpass ,motor_r_lowpass\n");
  // // fprintf(fp, str);
  // for (int i = 0; i < curr_p_index;  i++) {
  //   sprintf(str, "%d, %d, %f, %f, \n", p_counts_ls[i], p_counts_rs[i], p_cordinate_x[i], p_cordinate_y[i]);
  //   fprintf(fp, str);
  // }

  fclose(fp);


  // for (int i=0; i< curr_p_index; i++) {
  // sprintf(str, "%f, %f\n", simulate_x[i], simulate_y[i]);
  // // sprintf(str, "%d, %d\n", p_counts_rs[i], p_counts_ls[i]);
  // fprintf(fp, str);
  // }


  // for (int i=0; i< curr_p_index; i++) {
  // sprintf(str, "%d, %d, %f, %f, \n", p_counts_rs[i], p_counts_ls[i], p_cordinate_x[i], p_cordinate_y[i]);
  // // sprintf(str, "%d, %d\n", p_counts_rs[i], p_counts_ls[i]);
  // fprintf(fp, str);
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

  //for (int i=0; i<curr_index; i++) {
  //  sprintf(str, "%f\n", theta_[i]*180/M_PI);
  //  fprintf(fp, str);
  //}
}

 