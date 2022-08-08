#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_


#include <list>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tuple>
#include <unistd.h>
// #include <time.h>
#include "ctime"
#include "device_io.h"
#include "info_type.h"

// const int kCourseParamNum = 2000;

const int kCourseParamNum = 1133;

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
  //  double x = 0;
  //  double y = 0;

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

class P_WheelsControl {
 public:
  P_WheelsControl(MotorIo* motor_io);
  void P_exec(int32_t target_power_l, int32_t target_power_r);

 private:
  MotorIo* motor_io_;
  char f[256];
};

class PurePursuit {
  public:
   PurePursuit(MotorIo* motor_io, P_WheelsControl* p_wheels_control);
   double x=0, y=0, yaw;
   void Update(double x, double y);
   void read_trajectry_file_x();
   double target_distance = 0;
   double target_direction = 0;
   double difference_rad = 0;
   const double lf = 70;
   int base_p_power = 25; 
   double gain_kv_r = 0.057; //比例
   double gain_kv_l = 0.05; //比例
   double gain_kt_r = 6.0; //微分
   double gain_kt_l = 6.0; //微分


  //  float course_x[kCourseParamNum] = {0,6.01,12.02,18.03,24.04,30.05,36.06,42.07,48.08,54.09,60.1,66.11,72.12,78.13,84.14,90.15,96.16,102.17,108.18,114.19,120.2,126.21,132.22,138.23,144.24,150.25,156.26,162.27,168.28,174.29,180.3,186.31,192.32,198.33,204.34,210.35,216.36,222.37,228.38,234.39,240.4,246.41,252.42,258.43,264.44,270.45,276.46,282.47,288.48,294.49,300.5,306.51,312.52,318.53,324.54,330.55,336.56,342.57,348.58,354.59,360.6,366.61,372.62,378.63,384.64,390.65,396.66,402.67,408.68,414.69,420.7,426.71,432.72,438.73,444.74,450.75,456.76,462.77,468.78,474.79,480.8,486.81,492.82,498.83,504.84,510.85,516.86,522.87,528.88,534.89,540.9,546.91,552.92,558.93,564.94,570.95,576.96,582.97,588.98,594.99,601,607.01,613.02,619.03,625.04,631.05,637.06,643.07,649.08,655.09,661.1,667.11,673.12,679.13,685.14,691.15,697.16,703.17,709.18,715.19,721.2,727.21,733.22,739.23,745.24,751.25,757.26,763.27,769.28,775.29,781.3,787.31,793.32,799.33,805.34,811.35,817.36,823.37,829.38,835.39,841.4,847.41,853.42,859.43,865.44,871.45,877.46,883.47,889.48,895.49,901.5,907.51,913.52,919.53,925.54,931.55,937.56,943.57,949.58,955.59,961.6,967.61,973.62,979.63,985.64,991.65,997.66,1003.67,1009.68,1015.69,1021.7,1027.71,1033.72,1039.73,1045.74,1051.75,1057.76,1063.77,1069.78,1075.79,1081.8,1087.81,1093.82,1099.83,1105.84,1111.85,1117.86,1123.87,1129.88,1135.89,1141.9,1147.91,1153.92,1159.93,1165.94,1171.95,1177.96,1183.97,1189.98,1195.99,1202,1208.01,1214.02,1220.03,1226.04,1232.05,1238.06,1244.07,1250.08,1256.09,1262.1,1268.11,1274.12,1280.13,1286.14,1292.15,1298.16,1304.17,1310.18,1316.19,1322.2,1328.21,1334.22,1340.23,1346.24,1352.25,1358.26,1364.27,1370.28,1376.29,1382.3,1388.31,1394.32,1400.33,1406.34,1412.35,1418.36,1424.37,1430.38,1436.39,1442.4,1448.41,1454.42,1460.43,1466.44,1472.45,1478.46,1484.47,1490.48,1496.49,1502.5,1508.51,1514.52,1520.53,1526.54,1532.55,1538.56,1544.57,1550.58,1556.59,1562.6,1568.61,1574.62,1580.63,1586.64,1592.65,1598.66,1604.67,1604.67,1610.68,1616.69,1616.69,1622.7,1622.7,1628.71,1628.71,1628.71,1634.72,1634.72,1634.72,1640.73,1640.73,1640.73,1640.73,1640.73,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1652.75,1652.75,1652.75,1658.76,1658.76,1658.76,1664.77,1664.77,1670.78,1676.79,1676.79,1682.8,1688.81,1694.82,1700.83,1706.84,1712.85,1718.86,1724.87,1730.88,1736.89,1742.9,1748.91,1754.92,1760.93,1766.94,1772.95,1778.96,1784.97,1790.98,1796.99,1803,1809.01,1815.02,1821.03,1827.04,1833.05,1839.06,1845.07,1851.08,1857.09,1863.1,1869.11,1875.12,1881.13,1887.14,1893.15,1899.16,1905.17,1911.18,1917.19,1923.2,1929.21,1935.22,1941.23,1947.24,1953.25,1959.26,1965.27,1971.28,1977.29,1983.3,1989.31,1995.32,2001.33,2007.34,2013.35,2019.36,2025.37,2031.38,2037.39,2043.4,2049.41,2055.42,2061.43,2067.44,2073.45,2079.46,2085.47,2091.48,2097.49,2103.5,2109.51,2115.52,2121.53,2127.54,2133.55,2139.56,2145.57,2151.58,2157.59,2163.6,2169.61,2175.62,2181.63,2187.64,2193.65,2199.66,2205.67,2211.68,2217.69,2223.7,2229.71,2235.72,2241.73,2247.74,2253.75,2259.76,2265.77,2271.78,2277.79,2283.8,2289.81,2295.82,2301.83,2307.84,2313.85,2319.86,2325.87,2331.88,2337.89,2343.9,2349.91,2355.92,2361.93,2367.94,2373.95,2379.96,2385.97,2391.98,2397.99,2404,2410.01,2416.02,2422.03,2428.04,2434.05,2440.06,2446.07,2452.08,2458.09,2464.1,2470.11,2476.12,2482.13,2488.14,2494.15,2500.16,2506.17,2512.18,2518.19,2524.2,2530.21,2536.22,2542.23,2548.24,2554.25,2560.26,2566.27,2572.28,2578.29,2584.3,2590.31,2596.32,2602.33,2608.34,2614.35,2620.36,2626.37,2632.38,2638.39,2644.4,2650.41,2656.42,2662.43,2668.44,2674.45,2680.46,2686.47,2692.48,2698.49,2704.5,2710.51,2716.52,2722.53,2728.54,2734.55,2740.56,2746.57,2752.58,2758.59,2764.6,2770.61,2776.62,2782.63,2788.64,2794.65,2800.66,2806.67,2812.68,2818.69,2824.7,2830.71,2836.72,2842.73,2848.74,2854.75,2860.76,2866.77,2872.78,2878.79,2884.8,2890.81,2896.82,2902.83,2908.84,2914.85,2920.86,2926.87,2932.88,2938.89,2944.9,2950.91,2956.92,2962.93,2968.94,2974.95,2980.96,2986.97,2992.98,2998.99,3005,3011.01,3017.02,3023.03,3029.04,3035.05,3041.06,3047.07,3053.08,3059.09,3065.1,3071.11,3077.12,3083.13,3089.14,3095.15,3101.16,3107.17,3113.18,3119.19,3125.2,3131.21,3137.22,3143.23,3149.24,3155.25,3161.26,3167.27,3173.28,3179.29,3185.3,3191.31,3197.32,3203.33,3209.34,3215.35,3221.36,3227.37,3233.38,3239.39,3245.4,3251.41,3251.41,3257.42,3263.43,3269.44,3275.45,3275.45,3281.46,3287.47,3293.48,3293.48,3299.49,3299.49,3305.5,3311.51,3311.51,3317.52,3317.52,3323.53,3323.53,3329.54,3329.54,3335.55,3335.55,3335.55,3341.56,3341.56,3347.57,3347.57,3347.57,3353.58,3353.58,3353.58,3353.58,3359.59,3359.59,3359.59,3365.6,3365.6,3365.6,3365.6,3365.6,3365.6,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62};
  //  float course_y[kCourseParamNum] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6.01,6.01,6.01,6.01,12.02,12.02,18.03,18.03,18.03,24.04,30.05,30.05,36.06,42.07,42.07,48.08,54.09,60.1,66.11,72.12,78.13,84.14,90.15,96.16,102.17,108.18,114.19,120.2,126.21,132.22,138.23,144.24,150.25,156.26,162.27,168.28,174.29,180.3,186.31,192.32,198.33,204.34,210.35,216.36,222.37,228.38,234.39,240.4,246.41,252.42,258.43,264.44,270.45,276.46,282.47,288.48,294.49,300.5,306.51,312.52,318.53,324.54,330.55,336.56,342.57,348.58,354.59,360.6,366.61,372.62,378.63,384.64,390.65,396.66,402.67,408.68,414.69,420.7,426.71,432.72,438.73,444.74,450.75,456.76,462.77,468.78,474.79,480.8,486.81,492.82,498.83,504.84,510.85,516.86,522.87,528.88,534.89,540.9,546.91,552.92,558.93,564.94,570.95,576.96,582.97,588.98,594.99,601,607.01,613.02,619.03,625.04,631.05,637.06,643.07,649.08,655.09,661.1,667.11,673.12,679.13,685.14,691.15,697.16,703.17,709.18,715.19,721.2,727.21,733.22,739.23,745.24,751.25,757.26,763.27,769.28,775.29,781.3,787.31,793.32,799.33,805.34,811.35,817.36,823.37,829.38,835.39,841.4,847.41,853.42,859.43,865.44,871.45,877.46,883.47,889.48,895.49,901.5,907.51,913.52,919.53,925.54,931.55,937.56,943.57,949.58,955.59,961.6,967.61,973.62,979.63,985.64,991.65,997.66,1003.67,1009.68,1015.69,1021.7,1027.71,1033.72,1039.73,1045.74,1051.75,1057.76,1063.77,1069.78,1075.79,1081.8,1087.81,1093.82,1099.83,1105.84,1111.85,1117.86,1123.87,1129.88,1135.89,1141.9,1147.91,1153.92,1159.93,1165.94,1171.95,1177.96,1183.97,1189.98,1195.99,1202,1208.01,1214.02,1220.03,1226.04,1232.05,1238.06,1244.07,1250.08,1256.09,1262.1,1268.11,1274.12,1280.13,1286.14,1292.15,1298.16,1304.17,1310.18,1316.19,1322.2,1328.21,1334.22,1340.23,1346.24,1352.25,1358.26,1364.27,1370.28,1376.29,1382.3,1388.31,1394.32,1400.33,1406.34,1412.35,1418.36,1424.37,1430.38,1436.39,1442.4,1448.41,1454.42,1460.43,1466.44,1472.45,1478.46,1484.47,1490.48,1496.49,1502.5,1508.51,1514.52,1520.53,1526.54,1532.55,1538.56,1544.57,1550.58,1556.59,1562.6,1568.61,1574.62,1580.63,1586.64,1592.65,1598.66,1604.67,1610.68,1616.69,1622.7,1628.71,1634.72,1640.73,1646.74,1652.75,1658.76,1664.77,1670.78,1676.79,1682.8,1688.81,1694.82,1700.83,1706.84,1712.85,1718.86,1724.87,1730.88,1736.89,1742.9,1748.91,1754.92,1760.93,1766.94,1772.95,1778.96,1784.97,1790.98,1796.99,1803,1809.01,1815.02,1821.03,1827.04,1833.05,1839.06,1845.07,1851.08,1857.09,1863.1,1869.11,1875.12,1881.13,1887.14,1893.15,1899.16,1905.17,1911.18,1917.19,1923.2,1929.21,1935.22,1941.23,1947.24,1953.25,1959.26,1965.27,1971.28,1977.29,1983.3,1989.31,1995.32,2001.33,2007.34,2013.35,2019.36,2025.37,2031.38,2037.39,2043.4,2049.41,2055.42,2061.43,2067.44,2073.45,2079.46,2085.47,2091.48,2097.49,2103.5,2109.51,2115.52,2121.53,2127.54,2133.55,2139.56,2145.57,2151.58,2157.59,2163.6,2169.61,2175.62,2181.63,2181.63,2187.64,2187.64,2193.65,2193.65,2199.66,2199.66,2199.66,2205.67,2205.67,2205.67,2205.67,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2205.67,2205.67,2205.67,2205.67,2205.67,2205.67,2199.66,2199.66,2199.66,2199.66,2199.66,2193.65,2193.65,2193.65,2193.65,2187.64,2187.64,2187.64,2181.63,2181.63,2175.62,2175.62,2175.62,2169.61,2169.61,2163.6,2163.6,2157.59,2157.59,2151.58,2151.58,2145.57,2139.56,2139.56,2133.55,2133.55,2127.54,2121.53,2115.52,2115.52,2109.51,2103.5,2097.49,2091.48,2091.48,2085.47,2079.46,2073.45,2067.44,2061.43,2055.42,2049.41,2043.4,2037.39,2031.38,2025.37,2019.36,2013.35,2007.34,2001.33,1995.32,1989.31,1983.3,1977.29,1971.28,1965.27,1959.26,1953.25,1947.24,1941.23,1935.22,1929.21,1923.2,1917.19,1911.18,1905.17,1899.16,1893.15,1887.14,1881.13,1875.12,1869.11,1863.1,1857.09,1851.08,1845.07,1839.06,1833.05,1827.04,1821.03,1815.02,1809.01,1803,1796.99,1790.98,1784.97,1778.96,1772.95,1766.94,1760.93,1754.92,1748.91,1742.9,1736.89,1730.88,1724.87,1718.86,1712.85,1706.84,1700.83,1694.82,1688.81,1682.8,1676.79,1670.78,1664.77,1658.76,1652.75,1646.74,1640.73,1634.72,1628.71,1622.7,1616.69,1610.68,1604.67,1598.66,1592.65,1586.64,1580.63,1574.62,1568.61,1562.6,1556.59,1550.58,1544.57,1538.56,1532.55,1526.54,1520.53,1514.52,1508.51,1502.5,1496.49,1490.48,1484.47,1478.46,1472.45,1466.44,1460.43,1454.42,1448.41,1442.4,1436.39,1430.38,1424.37,1418.36,1412.35,1406.34,1400.33,1394.32,1388.31,1382.3,1376.29,1370.28,1364.27,1358.26,1352.25,1346.24,1340.23,1334.22,1328.21,1322.2,1316.19,1310.18,1304.17,1298.16,1292.15,1286.14,1280.13,1274.12,1268.11,1262.1,1256.09,1250.08,1244.07,1238.06,1232.05,1226.04,1220.03,1214.02,1208.01,1202,1195.99,1189.98,1183.97,1177.96,1171.95,1165.94,1159.93,1153.92,1147.91,1141.9,1135.89,1129.88,1123.87,1117.86,1111.85,1105.84,1099.83,1093.82,1087.81,1081.8,1075.79,1069.78,1063.77,1057.76,1051.75,1045.74,1039.73,1033.72,1027.71,1021.7,1015.69,1009.68,1003.67,997.66,991.65,985.64,979.63,973.62,967.61,961.6,955.59,949.58,943.57,937.56,931.55,925.54,919.53,913.52,907.51,901.5,895.49,889.48,883.47,877.46,871.45,865.44,859.43,853.42,847.41,841.4,835.39,829.38,823.37,817.36,811.35,805.34,799.33,793.32,787.31,781.3,775.29,769.28,763.27,757.26,751.25,745.24,739.23,733.22,727.21,721.2,715.19,709.18,703.17,697.16,691.15,685.14,679.13,673.12,667.11,661.1,655.09,649.08,643.07,637.06,631.05,625.04,619.03,613.02,607.01,601,594.99,588.98,582.97,576.96,570.95,564.94,558.93,552.92,546.91,540.9,534.89,528.88,522.87,516.86,510.85,504.84,498.83,492.82,486.81,480.8};

   float course_x[kCourseParamNum]={0,6.01,12.02,18.03,24.04,30.05,36.06,42.07,48.08,54.09,60.1,66.11,72.12,78.13,84.14,90.15,96.16,102.17,108.18,114.19,120.2,126.21,132.22,138.23,144.24,150.25,156.26,162.27,168.28,174.29,180.3,186.31,192.32,198.33,204.34,210.35,216.36,222.37,228.38,234.39,240.4,246.41,252.42,258.43,264.44,270.45,276.46,282.47,288.48,294.49,300.5,306.51,312.52,318.53,324.54,330.55,336.56,342.57,348.58,354.59,360.6,366.61,372.62,378.63,384.64,390.65,396.66,402.67,408.68,414.69,420.7,426.71,432.72,438.73,444.74,450.75,456.76,462.77,468.78,474.79,480.8,486.81,492.82,498.83,504.84,510.85,516.86,522.87,528.88,534.89,540.9,546.91,552.92,558.93,564.94,570.95,576.96,582.97,588.98,594.99,601,607.01,613.02,619.03,625.04,631.05,637.06,643.07,649.08,655.09,661.1,667.11,673.12,679.13,685.14,691.15,697.16,703.17,709.18,715.19,721.2,727.21,733.22,739.23,745.24,751.25,757.26,763.27,769.28,775.29,781.3,787.31,793.32,799.33,805.34,811.35,817.36,823.37,829.38,835.39,841.4,847.41,853.42,859.43,865.44,871.45,877.46,883.47,889.48,895.49,901.5,907.51,913.52,919.53,925.54,931.55,937.56,943.57,949.58,955.59,961.6,967.61,973.62,979.63,985.64,991.65,997.66,1003.67,1009.68,1015.69,1021.7,1027.71,1033.72,1039.73,1045.74,1051.75,1057.76,1063.77,1069.78,1075.79,1081.8,1087.81,1093.82,1099.83,1105.84,1111.85,1117.86,1123.87,1129.88,1135.89,1141.9,1147.91,1153.92,1159.93,1165.94,1171.95,1177.96,1183.97,1189.98,1195.99,1202,1208.01,1214.02,1220.03,1226.04,1232.05,1238.06,1244.07,1250.08,1256.09,1262.1,1268.11,1274.12,1280.13,1286.14,1292.15,1298.16,1304.17,1310.18,1316.19,1322.2,1328.21,1334.22,1340.23,1346.24,1352.25,1358.26,1364.27,1370.28,1376.29,1382.3,1388.31,1394.32,1400.33,1406.34,1412.35,1418.36,1424.37,1430.38,1436.39,1442.4,1448.41,1454.42,1460.43,1466.44,1472.45,1478.46,1484.47,1490.48,1496.49,1502.5,1508.51,1514.52,1520.53,1526.54,1532.55,1538.56,1544.57,1550.58,1556.59,1562.6,1568.61,1574.62,1580.63,1586.64,1592.65,1598.66,1604.67,1604.67,1610.68,1616.69,1616.69,1622.7,1622.7,1628.71,1628.71,1628.71,1634.72,1634.72,1634.72,1640.73,1640.73,1640.73,1640.73,1640.73,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1648.96532,1651.19064,1653.41596,1655.64128,1657.8666,1660.09192,1662.31724,1664.54256,1666.76788,1668.9932,1671.21852,1673.44384,1675.66916,1677.89448,1680.1198,1682.34512,1684.57044,1686.79576,1689.02108,1691.2464,1693.47172,1695.69704,1697.92236,1700.14768,1702.373,1704.59832,1706.82364,1709.04896,1711.27428,1713.4996,1715.72492,1717.95024,1720.17556,1722.40088,1724.6262,1726.85152,1729.07684,1731.30216,1733.52748,1735.7528,1737.97812,1740.20344,1742.42876,1744.65408,1746.8794,1749.10472,1751.33004,1753.55536,1755.78068,1758.006,1760.23132,1762.45664,1764.68196,1766.90728,1769.1326,1771.35792,1773.58324,1775.80856,1778.03388,1780.2592,1782.48452,1784.70984,1786.93516,1789.16048,1791.3858,1793.61112,1795.83644,1798.06176,1800.28708,1802.5124,1804.73772,1806.96304,1809.18836,1811.41368,1813.639,1815.86432,1818.08964,1820.31496,1822.54028,1824.7656,1826.99092,1829.21624,1831.44156,1833.66688,1835.8922,1838.11752,1840.34284,1842.56816,1844.79348,1847.0188,1849.24412,1851.46944,1853.69476,1855.92008,1858.1454,1860.37072,1862.59604,1864.82136,1867.04668,1869.272,1871.49732,1873.72264,1875.94796,1878.17328,1880.3986,1882.62392,1884.84924,1887.07456,1889.29988,1891.5252,1893.75052,1895.97584,1898.20116,1900.42648,1902.6518,1904.87712,1907.10244,1909.32776,1911.55308,1913.7784,1916.00372,1918.22904,1920.45436,1922.67968,1924.905,1927.13032,1929.35564,1931.58096,1933.80628,1936.0316,1938.25692,1940.48224,1942.70756,1944.93288,1947.1582,1949.38352,1951.60884,1953.83416,1956.05948,1958.2848,1960.51012,1962.73544,1964.96076,1967.18608,1969.4114,1971.63672,1973.86204,1976.08736,1978.31268,1980.538,1982.76332,1984.98864,1987.21396,1989.43928,1991.6646,1993.88992,1996.11524,1998.34056,2000.56588,2002.7912,2005.01652,2007.24184,2009.46716,2011.69248,2013.9178,2016.14312,2018.36844,2020.59376,2022.81908,2025.0444,2027.26972,2029.49504,2031.72036,2033.94568,2036.171,2038.39632,2040.62164,2042.84696,2045.07228,2047.2976,2049.52292,2051.74824,2053.97356,2056.19888,2058.4242,2060.64952,2062.87484,2065.10016,2067.32548,2069.5508,2071.77612,2074.00144,2076.22676,2078.45208,2080.6774,2082.90272,2085.12804,2087.35336,2089.57868,2091.804,2094.02932,2096.25464,2098.47996,2100.70528,2102.9306,2105.15592,2107.38124,2109.60656,2111.83188,2114.0572,2116.28252,2118.50784,2120.73316,2122.95848,2125.1838,2127.40912,2129.63444,2131.85976,2134.08508,2136.3104,2138.53572,2140.76104,2142.98636,2145.21168,2147.437,2149.66232,2151.88764,2154.11296,2156.33828,2158.5636,2160.78892,2163.01424,2165.23956,2167.46488,2169.6902,2171.91552,2174.14084,2176.36616,2178.59148,2180.8168,2183.04212,2185.26744,2187.49276,2189.71808,2191.9434,2194.16872,2196.39404,2198.61936,2200.84468,2203.07,2205.29532,2207.52064,2209.74596,2211.97128,2214.1966,2216.42192,2218.64724,2220.87256,2223.09788,2225.3232,2227.54852,2229.77384,2231.99916,2234.22448,2236.4498,2238.67512,2240.90044,2243.12576,2245.35108,2247.5764,2249.80172,2252.02704,2254.25236,2256.47768,2258.703,2260.92832,2263.15364,2265.37896,2267.60428,2269.8296,2272.05492,2274.28024,2276.50556,2278.73088,2280.9562,2283.18152,2285.40684,2287.63216,2289.85748,2292.0828,2294.30812,2296.53344,2298.75876,2300.98408,2303.2094,2305.43472,2307.66004,2309.88536,2312.11068,2314.336,2316.56132,2318.78664,2321.01196,2323.23728,2325.4626,2327.68792,2329.91324,2332.13856,2334.36388,2336.5892,2338.81452,2341.03984,2343.26516,2345.49048,2347.7158,2349.94112,2352.16644,2354.39176,2356.61708,2358.8424,2361.06772,2363.29304,2365.51836,2367.74368,2369.969,2372.19432,2374.41964,2376.64496,2378.87028,2381.0956,2383.32092,2385.54624,2387.77156,2389.99688,2392.2222,2394.44752,2396.67284,2398.89816,2401.12348,2403.3488,2405.57412,2407.79944,2410.02476,2412.25008,2414.4754,2416.70072,2418.92604,2421.15136,2423.37668,2425.602,2427.82732,2430.05264,2432.27796,2434.50328,2436.7286,2438.95392,2441.17924,2443.40456,2445.62988,2447.8552,2450.08052,2452.30584,2454.53116,2456.75648,2458.9818,2461.20712,2463.43244,2465.65776,2467.88308,2470.1084,2476.12,2482.13,2488.14,2494.15,2500.16,2506.17,2512.18,2518.19,2524.2,2530.21,2536.22,2542.23,2548.24,2554.25,2560.26,2566.27,2572.28,2578.29,2584.3,2590.31,2596.32,2602.33,2608.34,2614.35,2620.36,2626.37,2632.38,2638.39,2644.4,2650.41,2656.42,2662.43,2668.44,2674.45,2680.46,2686.47,2692.48,2698.49,2704.5,2710.51,2716.52,2722.53,2728.54,2734.55,2740.56,2746.57,2752.58,2758.59,2764.6,2770.61,2776.62,2782.63,2788.64,2794.65,2800.66,2806.67,2812.68,2818.69,2824.7,2830.71,2836.72,2842.73,2848.74,2854.75,2860.76,2866.77,2872.78,2878.79,2884.8,2890.81,2896.82,2902.83,2908.84,2914.85,2920.86,2926.87,2932.88,2938.89,2944.9,2950.91,2956.92,2962.93,2968.94,2974.95,2980.96,2986.97,2992.98,2998.99,3005,3011.01,3017.02,3023.03,3029.04,3035.05,3041.06,3047.07,3053.08,3059.09,3065.1,3071.11,3077.12,3083.13,3089.14,3095.15,3101.16,3107.17,3113.18,3119.19,3125.2,3131.21,3137.22,3143.23,3149.24,3155.25,3161.26,3167.27,3173.28,3179.29,3185.3,3191.31,3197.32,3203.33,3209.34,3215.35,3221.36,3227.37,3233.38,3239.39,3245.4,3251.41,3251.41,3257.42,3263.43,3269.44,3275.45,3275.45,3281.46,3287.47,3293.48,3293.48,3299.49,3299.49,3305.5,3311.51,3311.51,3317.52,3317.52,3323.53,3323.53,3329.54,3329.54,3335.55,3335.55,3335.55,3341.56,3341.56,3347.57,3347.57,3347.57,3353.58,3353.58,3353.58,3353.58,3359.59,3359.59,3359.59,3365.6,3365.6,3365.6,3365.6,3365.6,3365.6,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62};
   float course_y[kCourseParamNum]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3.6849,7.3698,11.0547,14.7396,18.4245,22.1094,25.7943,29.4792,33.1641,36.849,40.5339,44.2188,47.9037,51.5886,55.2735,58.9584,62.6433,66.3282,70.0131,73.698,77.3829,81.0678,84.7527,88.4376,92.1225,95.8074,99.4923,103.1772,106.8621,110.547,114.2319,117.9168,121.6017,125.2866,128.9715,132.6564,136.3413,140.0262,143.7111,147.396,151.0809,154.7658,158.4507,162.1356,165.8205,169.5054,173.1903,176.8752,180.5601,184.245,187.9299,191.6148,195.2997,198.9846,202.6695,206.3544,210.0393,213.7242,217.4091,221.094,224.7789,228.4638,232.1487,235.8336,239.5185,243.2034,246.8883,250.5732,254.2581,257.943,261.6279,265.3128,268.9977,272.6826,276.3675,280.0524,283.7373,287.4222,291.1071,294.792,298.4769,302.1618,305.8467,309.5316,313.2165,316.9014,320.5863,324.2712,327.9561,331.641,335.3259,339.0108,342.6957,346.3806,350.0655,353.7504,357.4353,361.1202,364.8051,368.49,372.1749,375.8598,379.5447,383.2296,386.9145,390.5994,394.2843,397.9692,401.6541,405.339,409.0239,412.7088,416.3937,420.0786,423.7635,427.4484,431.1333,434.8182,438.5031,442.188,445.8729,449.5578,453.2427,456.9276,460.6125,464.2974,467.9823,471.6672,475.3521,479.037,482.7219,486.4068,490.0917,493.7766,497.4615,501.1464,504.8313,510.85,516.86,522.87,528.88,534.89,540.9,546.91,552.92,558.93,564.94,570.95,576.96,582.97,588.98,594.99,601,607.01,613.02,619.03,625.04,631.05,637.06,643.07,649.08,655.09,661.1,667.11,673.12,679.13,685.14,691.15,697.16,703.17,709.18,715.19,721.2,727.21,733.22,739.23,745.24,751.25,757.26,763.27,769.28,775.29,781.3,787.31,793.32,799.33,805.34,811.35,817.36,823.37,829.38,835.39,841.4,847.41,853.42,859.43,865.44,871.45,877.46,883.47,889.48,895.49,901.5,907.51,913.52,919.53,925.54,931.55,937.56,943.57,949.58,955.59,961.6,967.61,973.62,979.63,985.64,991.65,997.66,1003.67,1009.68,1015.69,1021.7,1027.71,1033.72,1039.73,1045.74,1051.75,1057.76,1063.77,1069.78,1075.79,1081.8,1087.81,1093.82,1099.83,1105.84,1111.85,1117.86,1123.87,1129.88,1135.89,1141.9,1147.91,1153.92,1159.93,1165.94,1171.95,1177.96,1183.97,1189.98,1195.99,1202,1208.01,1214.02,1220.03,1226.04,1232.05,1238.06,1244.07,1250.08,1256.09,1262.1,1268.11,1274.12,1280.13,1286.14,1292.15,1298.16,1304.17,1310.18,1316.19,1322.2,1328.21,1334.22,1340.23,1346.24,1352.25,1358.26,1364.27,1370.28,1376.29,1382.3,1388.31,1394.32,1400.33,1406.34,1412.35,1418.36,1424.37,1430.38,1436.39,1442.4,1448.41,1454.42,1460.43,1466.44,1472.45,1478.46,1484.47,1490.48,1496.49,1502.5,1508.51,1514.52,1520.53,1526.54,1532.55,1538.56,1544.57,1550.58,1556.59,1562.6,1568.61,1574.62,1580.63,1586.64,1592.65,1598.66,1604.67,1610.68,1616.69,1622.7,1628.71,1634.72,1640.73,1646.74,1652.75,1658.76,1664.77,1670.78,1676.79,1682.8,1688.81,1694.82,1700.83,1706.84,1712.85,1718.86,1724.87,1730.88,1736.89,1742.9,1748.91,1754.92,1760.93,1766.94,1772.95,1778.96,1784.97,1790.98,1796.99,1803,1809.01,1815.02,1821.03,1827.04,1833.05,1839.06,1845.07,1851.08,1857.09,1863.1,1869.11,1875.12,1881.13,1887.14,1893.15,1899.16,1905.17,1911.18,1917.19,1923.2,1929.21,1935.22,1941.23,1947.24,1953.25,1959.26,1965.27,1971.28,1977.29,1983.3,1989.31,1995.32,2001.33,2007.34,2013.35,2019.36,2025.37,2031.38,2037.39,2043.4,2049.41,2055.42,2061.43,2067.44,2073.45,2079.46,2085.47,2091.48,2097.49,2103.5,2109.51,2115.52,2121.53,2127.54,2133.55,2139.56,2145.57,2151.58,2157.59,2163.6,2169.61,2175.62,2181.63,2181.63,2187.64,2187.64,2193.65,2193.65,2199.66,2199.66,2199.66,2205.67,2205.67,2205.67,2205.67,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2205.67,2205.67,2205.67,2205.67,2205.67,2205.67,2199.66,2199.66,2199.66,2199.66,2199.66,2193.65,2193.65,2193.65,2193.65,2187.64,2187.64,2187.64,2181.63,2181.63,2175.62,2175.62,2175.62,2169.61,2169.61,2163.6,2163.6,2157.59,2157.59,2151.58,2151.58,2145.57,2139.56,2139.56,2133.55,2133.55,2127.54,2121.53,2115.52,2115.52,2109.51,2103.5,2097.49,2091.48,2091.48,2085.47,2079.46,2073.45,2067.44,2061.43,2055.42,2049.41,2043.4,2037.39,2031.38,2025.37,2019.36,2013.35,2007.34,2001.33,1995.32,1989.31,1983.3,1977.29,1971.28,1965.27,1959.26,1953.25,1947.24,1941.23,1935.22,1929.21,1923.2,1917.19,1911.18,1905.17,1899.16,1893.15,1887.14,1881.13,1875.12,1869.11,1863.1,1857.09,1851.08,1845.07,1839.06,1833.05,1827.04,1821.03,1815.02,1809.01,1803,1796.99,1790.98,1784.97,1778.96,1772.95,1766.94,1760.93,1754.92,1748.91,1742.9,1736.89,1730.88,1724.87,1718.86,1712.85,1706.84,1700.83,1694.82,1688.81,1682.8,1676.79,1670.78,1664.77,1658.76,1652.75,1646.74,1640.73,1634.72,1628.71,1622.7,1616.69,1610.68,1604.67,1598.66,1592.65,1586.64,1580.63,1574.62,1568.61,1562.6,1556.59,1550.58,1544.57,1538.56,1532.55,1526.54,1520.53,1514.52,1508.51,1502.5,1496.49,1490.48,1484.47,1478.46,1472.45,1466.44,1460.43,1454.42,1448.41,1442.4,1436.39,1430.38,1424.37,1418.36,1412.35,1406.34,1400.33,1394.32,1388.31,1382.3,1376.29,1370.28,1364.27,1358.26,1352.25,1346.24,1340.23,1334.22,1328.21,1322.2,1316.19,1310.18,1304.17,1298.16,1292.15,1286.14,1280.13,1274.12,1268.11,1262.1,1256.09,1250.08,1244.07,1238.06,1232.05,1226.04,1220.03,1214.02,1208.01,1202,1195.99,1189.98,1183.97,1177.96,1171.95,1165.94,1159.93,1153.92,1147.91,1141.9,1135.89,1129.88,1123.87,1117.86,1111.85,1105.84,1099.83,1093.82,1087.81,1081.8,1075.79,1069.78,1063.77,1057.76,1051.75,1045.74,1039.73,1033.72,1027.71,1021.7,1015.69,1009.68,1003.67,997.66,991.65,985.64,979.63,973.62,967.61,961.6,955.59,949.58,943.57,937.56,931.55,925.54,919.53,913.52,907.51,901.5,895.49,889.48,883.47,877.46,871.45,865.44,859.43,853.42,847.41,841.4,835.39,829.38,823.37,817.36,811.35,805.34,799.33,793.32,787.31,781.3,775.29,769.28,763.27,757.26,751.25,745.24,739.23,733.22,727.21,721.2,715.19,709.18,703.17,697.16,691.15,685.14,679.13,673.12,667.11,661.1,655.09,649.08,643.07,637.06,631.05,625.04,619.03,613.02,607.01,601,594.99,588.98,582.97,576.96,570.95,564.94,558.93,552.92,546.91,540.9,534.89,528.88,522.87,516.86,510.85,504.84,498.83,492.82,486.81,480.8};


  private:
   double calc_distance(double point_x, double point_y);
   void readTargetCourseCoordinate();
   std::tuple<int, double> pursuit_control(int ind);
   std::tuple<int, double> search_target_index();
   MotorIo* motor_io_; 
   Odometry *odometry_;
   P_WheelsControl* p_wheels_control_;

   int ind;
   int target_ind;
   int pre_point_index= INT_MAX;
   double turning_radius;
  //  double p_ll;
  //  double p_lr;
  //  double p_d = 126;
   double p_power_l;
   double p_power_r;
  //  int v = 50;
   char str [256],a[256],b[256],c[256],d[256];
   
   double direction_odo;
   double p_lf;
   double delta = 0;

  //  double omega;
  //  double para = 100;  //  const float course_x[kCourseParamNum] = {};

  //  const float course_y[kCourseParamNum] = {};
   
  //  CubicSpline* cubic_spline_;
};

class Localize {
 public:
  Localize(MotorIo* motor_io, P_WheelsControl* p_wheels_control);
  void Update();
  void SaveOdometry();

 private:
  Odometry* odometry_;
  PurePursuit* pure_pursuit_;
  int curr_p_index = 0;
  double distance_ = 0;
  double odometry_x = 0;
  double odometry_y = 0;
  double simu_x = 0;
  double simu_y = 0;
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
  char str [256], str_[256];
  double real_distance = 0;
  int base_p_power_ = 0;
  double lf_ = 0;
  double gain_kv_r_ = 0;
  double gain_kv_l_ = 0;
  double gain_kt_r_ = 0;
  double gain_kt_l_ = 0;


  // clock_t before_time = 0;
  //char str[264];
  // struct timespec now_time;
};

#endif  // ETRC22_ETRC_INFO_H_
