#ifndef _stdio_H_
  #include <stdio.h>
#endif
#ifndef _stdlib_H_
  #include <stdlib.h>
#endif
#ifndef _math_H_
  #include <math.h>
#endif
#ifndef _stdbool_H_
  #include <stdbool.h>
#endif
#ifndef _stdbool_H_
  #include <stdbool.h>
#endif
#ifndef _string_H_
  #include <string.h>
#endif

#define Rectangle_Probe

#if ENABLED(DEBUG_LEVELING_FEATURE)
void print_xyz(const char *prefix, const char *suffix, const float x, const float y, const float z)
{
  serialprintPGM(prefix);
  SERIAL_CHAR('(');
  SERIAL_ECHO(x);
  SERIAL_ECHOPAIR(", ", y);
  SERIAL_ECHOPAIR(", ", z);
  SERIAL_CHAR(')');
  if (suffix) serialprintPGM(suffix);
  else
    SERIAL_EOL();
}

void print_Joint(const char *prefix, const char *suffix, const long j1, const long j2, const long j3, const long j4, const long j5)
{
  serialprintPGM(prefix);
  SERIAL_CHAR('(');
  SERIAL_ECHO(j1);
  SERIAL_ECHOPAIR(", ", j2);
  SERIAL_ECHOPAIR(", ", j3);
  SERIAL_ECHOPAIR(", ", j4);
  SERIAL_ECHOPAIR(", ", j5);
  SERIAL_CHAR(')');
  if (suffix) serialprintPGM(suffix);
  else
    SERIAL_EOL();
}

void print_xyz(const char *prefix, const char *suffix, const float xyz[4]) { print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]); }

void print_Joint(const char *prefix, const char *suffix, const long Joint_POS[5])
{
  print_Joint(prefix, suffix, Joint_POS[Joint1_AXIS], Joint_POS[Joint2_AXIS], Joint_POS[Joint3_AXIS], Joint_POS[Joint4_AXIS], Joint_POS[Joint5_AXIS]);
}

  #define DEBUG_POS(SUFFIX, VAR)                                                                                                                     \
    do                                                                                                                                               \
    {                                                                                                                                                \
      print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR);                                                                        \
    } while (0)

  #define DEBUG_POS_Joint(SUFFIX, VAR)                                                                                                               \
    do                                                                                                                                               \
    {                                                                                                                                                \
      print_Joint(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR);                                                                      \
    } while (0)
#endif

int32_t HOME_position_Joint_Mesh[25][5] = {
    {2241, 2643, 20041, 3199, -2126}, {256, 2616, 20379, 3199, -2029}, {-1760, 2633, 20163, 3199, -2091}, {-4302, 2391, 23956, 0, 567},
    {-5935, 2443, 22889, 0, 927},     {-5139, 2603, 20540, 0, 1644},   {-3691, 2529, 21541, 0, 1349},     {-2096, 2489, 22133, 0, 1168},
    {-416, 2479, 22299, 0, 1115},     {1258, 2495, 22037, 0, 1197},    {1031, 2704, 19316, 0, 1989},      {-408, 2682, 19567, 0, 1919},
    {-1851, 2696, 19407, 0, 1963},    {-3240, 2745, 18838, 0, 2119},   {-4531, 2833, 17859, 0, 2380},     {-4055, 3142, 14715, 0, 3178},
    {-2895, 3038, 15733, 0, 2926},    {-1666, 2981, 16314, 0, 2780},   {-402, 2965, 16476, 0, 2739},      {861, 2990, 16222, 0, 2803},
    {727, 3376, 12490, 0, 3714},      {-397, 3346, 12775, 0, 3647},    {-1522, 3365, 12594, 0, 3690},     {-2622, 3436, 11935, 0, 3845},
    {-3673, 3568, 10742, 0, 4124},
};

int current_Probe_position = 67;
int Probe_pointx = 0, Probe_pointy = 0;

int32_t ZERO_position_Joint[Joint_All] = {2241, 2643, 20041, 3199, -2126};
float ZERO_position[XYZE] = {0, 0, 0.25, 0};

int32_t HOME_position_Joint[Joint_All] = {0, 0, 0, 0, 0};
float HOME_position[XYZE] = {-45.484, -10, 0, current_position[E_AXIS] - 3};

// int32_t HOME_position_Z20_Joint[Joint_All]={-609, 3504, 16908, 0, -2917};
int32_t HOME_position_Z20_Joint[Joint_All] = {4506, 19059, 101482, 85898, -5163};
float HOME_position_Z20[XYZE] = {0, 0, 20, 0};
/*
int32_t HOME_position_Z10_Joint[Joint_All]={-609, 3621, 16573, 0, -2947};
float   HOME_position_Z10[XYZE]={0, 0, 10, 0};
int32_t HOME_position_Z5_Joint[Joint_All]={-609, 3680, 16396, 0, -2960};
float   HOME_position_Z5[XYZE]={0, 0, 5, 0};
int32_t HOME_position_Z0_Joint[Joint_All]={-609, 3739, 16214, 0, -2972};
float   HOME_position_Z0[XYZE]={0, 0, 0, 0};
int32_t HOME_position_ZNeg10_Joint[Joint_All]={-609, 3857, 15830, 0, -2994};
float   HOME_position_ZNeg10[XYZE]={0, 0, -10, 0};
float   HOME_position_Slope[Joint_All]={0,	0.1175,	-0.347,	0,  -0.0275};
//*/

float a[5] = {0.0000000000, 0.0000017500, -0.0000560000, 0.0000000000, 0.0000111250};
float b[5] = {0.0000000000, -1.0184999704, 0.9434999824, 0.0000000000, 0.3357500136};
float c[5] = {128.0000000000, 20845.0000000000, 104799.0000000000, -23.0000000000, 3630.0000000000};
// float a_m[25][5];
// float b_m[25][5];
// float c_m[25][5];
const PROGMEM float Probe_position[300] = {
    0.00000,   0.00000,   0, 100.00000, 0.00000,   0, 200.00000,  71.21822,  0, 300.00000, 22.86693,  0, 400.00000, 1.82195,   0,
    500.00000, 4.11011,   0, 600.00000, 30.12197,  0, 700.00000,  85.03521,  0, 800.00000, 0.00000,   0, 880.00000, 0.00000,   0,
    0.00000,   100.00000, 0, 100.00000, 160.71520, 0, 200.00000,  100.00000, 0, 300.00000, 100.00000, 0, 400.00000, 100.00000, 0,
    500.00000, 100.00000, 0, 600.00000, 100.00000, 0, 700.00000,  100.00000, 0, 800.00000, 187.01779, 0, 880.00000, 100.00000, 0,
    71.21822,  200.00000, 0, 100.00000, 200.00000, 0, 200.00000,  200.00000, 0, 300.00000, 200.00000, 0, 400.00000, 200.00000, 0,
    500.00000, 200.00000, 0, 600.00000, 200.00000, 0, 700.00000,  200.00000, 0, 800.00000, 200.00000, 0, 808.78178, 200.00000, 0,
    22.86693,  300.00000, 0, 100.00000, 300.00000, 0, 200.00000,  300.00000, 0, 300.00000, 300.00000, 0, 400.00000, 269.63275, 0,
    500.00000, 275.60718, 0, 600.00000, 300.00000, 0, 700.00000,  300.00000, 0, 800.00000, 300.00000, 0, 857.13307, 300.00000, 0,
    1.82195,   400.00000, 0, 100.00000, 400.00000, 0, 200.00000,  400.00000, 0, 300.00000, 335.00000, 0, 400.00000, 400.00000, 0,
    500.00000, 400.00000, 0, 600.00000, 369.11277, 0, 700.00000,  400.00000, 0, 800.00000, 400.00000, 0, 878.17805, 400.00000, 0,
    4.11011,   500.00000, 0, 100.00000, 500.00000, 0, 200.00000,  500.00000, 0, 300.00000, 545.00000, 0, 400.00000, 500.00000, 0,
    500.00000, 500.00000, 0, 600.00000, 510.88723, 0, 700.00000,  500.00000, 0, 800.00000, 500.00000, 0, 875.88989, 500.00000, 0,
    30.12197,  600.00000, 0, 100.00000, 600.00000, 0, 209.837014, 615.00000, 0, 300.00000, 615.00000, 0, 400.00000, 615.00000, 0,
    500.00000, 615.00000, 0, 600.00000, 615.00000, 0, 670.16299,  615.00000, 0, 800.00000, 600.00000, 0, 849.87803, 600.00000, 0,
    85.03521,  700.00000, 0, 100.00000, 700.00000, 0, 209.837014, 715.00000, 0, 300.00000, 715.00000, 0, 400.00000, 715.00000, 0,
    500.00000, 715.00000, 0, 600.00000, 715.00000, 0, 670.16299,  715.00000, 0, 800.00000, 692.98221, 0, 880.00000, 700.00000, 0,
    0.00000,   800.00000, 0, 100.00000, 719.28480, 0, 209.837014, 815.00000, 0, 300.00000, 815.00000, 0, 400.00000, 815.00000, 0,
    500.00000, 815.00000, 0, 600.00000, 815.00000, 0, 670.16299,  815.00000, 0, 800.00000, 800.00000, 0, 880.00000, 800.00000, 0,
    0.00000,   880.00000, 0, 100.00000, 880.00000, 0, 200.00000,  808.78178, 0, 300.00000, 857.13307, 0, 400.00000, 878.17805, 0,
    500.00000, 875.88989, 0, 600.00000, 849.87803, 0, 700.00000,  794.96479, 0, 800.00000, 880.00000, 0, 880.00000, 880.00000, 0};

const PROGMEM float Probe_position_ADD[39] = {160.71520, 100.00000, 0, 719.28480, 100.00000, 0, 335.00000, 300.00000, 0, 545.00000, 300.00000, 0,
                                              269.63275, 400.00000, 0, 610.36725, 400.00000, 0, 275.60718, 500.00000, 0, 604.39282, 500.00000, 0,
                                              369.11277, 600.00000, 0, 510.88723, 600.00000, 0, 794.96479, 700.00000, 0, 187.01779, 800.00000, 0,
                                              692.98221, 800.00000, 0};

const PROGMEM float a_m1[125] = {
    0.0000000000,  0.0000026250, -0.0000550000, 0.0000000000, -0.0000106250, 0.0000000000,  0.0000015000, -0.0000561250, -0.0000001250, -0.0000113750,
    0.0000000000,  0.0000010000, -0.0000567500, 0.0000000000, -0.0000118750, 0.0000000000,  0.0000015000, -0.0000565000, 0.0000000000,  -0.0000113750,
    0.0000000000,  0.0000027500, -0.0000558750, 0.0000000000, -0.0000107500, 0.0000001250,  0.0000055000, -0.0000555000, -0.0000001250, -0.0000090000,
    0.0000000000,  0.0000047500, -0.0000551250, 0.0000000000, -0.0000093750, 0.0000000000,  0.0000043750, -0.0000550000, 0.0000000000,  -0.0000095000,
    0.0000000000,  0.0000050000, -0.0000556250, 0.0000000000, -0.0000093750, 0.0000001250,  0.0000053750, -0.0000552500, 0.0000000000,  -0.0000090000,
    0.0000000000,  0.0000033750, -0.0000553750, 0.0000000000, 0.0000103750,  0.0000000000,  0.0000023750, -0.0000555000, 0.0000000000,  0.0000106250,
    0.0000000000,  0.0000022500, -0.0000558750, 0.0000000000, 0.0000108750,  0.0000000000,  0.0000025000, -0.0000555000, 0.0000001250,  0.0000106250,
    -0.0000001250, 0.0000035000, -0.0000556250, 0.0000000000, 0.0000101250,  0.0000000000,  0.0000061250, -0.0000562500, 0.0000000000,  0.0000088750,
    0.0000000000,  0.0000055000, -0.0000557500, 0.0000000000, 0.0000090000,  0.0000000000,  0.0000053750, -0.0000551250, 0.0000000000,  0.0000090000,
    0.0000000000,  0.0000053750, -0.0000557500, 0.0000000000, 0.0000090000,  0.0000000000,  0.0000060000, -0.0000560000, 0.0000000000,  0.0000088750,
    0.0000000000,  0.0000090000, -0.0000613750, 0.0000000000, 0.0000086250,  0.0000000000,  0.0000085000, -0.0000601250, 0.0000000000,  0.0000086250,
    0.0000000000,  0.0000081250, -0.0000591250, 0.0000000000, 0.0000085000,  -0.0000001250, 0.0000082500, -0.0000596250, 0.0000000000,  0.0000085000,
    0.0000000000,  0.0000088750, -0.0000622500, 0.0000000000, 0.0000086250};

const PROGMEM float b_m1[125] = {
    0.0000000000,  -0.9827499986, 0.9254999757, 0.0000000000, -0.3207499981, 0.0000000000,  -1.0255000591, 0.9477499723, 0.0002500000, -0.3387500048,
    0.0000000000,  -1.0414999723, 0.9564999938, 0.0000000000, -0.3452500105, 0.0000000000,  -1.0255000591, 0.9474999905, 0.0000000000, -0.3387500048,
    0.0000000000,  -0.9829999804, 0.9252499938, 0.0000000000, -0.3210000098, -0.0002500000, -0.8815000057, 0.8939999938, 0.0002500000, -0.2739999890,
    0.0000000000,  -0.9075000286, 0.8977500200, 0.0000000000, -0.2867499888, 0.0000000000,  -0.9167500138, 0.8999999762, 0.0000000000, -0.2915000021,
    0.0000000000,  -0.9075000286, 0.8977500200, 0.0000000000, -0.2867499888, 0.0002500000,  -0.8817499876, 0.8939999938, 0.0000000000, -0.2739999890,
    0.0000000000,  -0.9627500176, 0.9162499905, 0.0000000000, 0.3122499883,  0.0000000000,  -0.9912499785, 0.9290000200, 0.0000000000, 0.3247500062,
    0.0000000000,  -1.0015000105, 0.9342499971, 0.0000000000, 0.3287500143,  0.0000000000,  -0.9909999967, 0.9294999838, 0.0002500000, 0.3242500126,
    -0.0002500000, -0.9629999995, 0.9162499905, 0.0000000000, 0.3122499883,  0.0000000000,  -0.8642500043, 0.8945000172, 0.0000000000, 0.2647500038,
    0.0000000000,  -0.8820000291, 0.8939999938, 0.0000000000, 0.2739999890,  0.0000000000,  -0.8877500296, 0.8942499757, 0.0000000000, 0.2775000036,
    0.0000000000,  -0.8817499876, 0.8945000172, 0.0000000000, 0.2739999890,  0.0000000000,  -0.8644999862, 0.8945000172, 0.0000000000, 0.2647500038,
    0.0000000000,  -0.7990000248, 0.9452499747, 0.0000000000, 0.2192499936,  0.0000000000,  -0.8080000281, 0.9277499914, 0.0000000000, 0.2277500033,
    0.0000000000,  -0.8112499714, 0.9232500196, 0.0000000000, 0.2304999977,  -0.0002500000, -0.8080000281, 0.9287499785, 0.0000000000, 0.2275000066,
    0.0000000000,  -0.7987499833, 0.9440000057, 0.0000000000, 0.2192499936};

const PROGMEM float c_m1[125] = {
    4503.0000000000,   21265.0000000000,  101258.0000000000, 85898.0000000000,  -4295.0000000000,  2489.0000000000,   20772.0000000000,
    105454.0000000000, 85898.0000000000,  -3650.0000000000,  249.0000000000,    20616.0000000000,  106887.0000000000, 85898.0000000000,
    -3423.0000000000,  -2019.0000000000,  20772.0000000000,  105455.0000000000, 85898.0000000000,  -3650.0000000000,  -4110.0000000000,
    21265.0000000000,  101261.0000000000, 85898.0000000000,  -4295.0000000000,  -3478.0000000000,  23070.0000000000,  88137.0000000000,
    85898.0000000000,  -6175.0000000000,  -1720.0000000000,  22498.0000000000,  92042.0000000000,  85898.0000000000,  -5634.0000000000,
    138.0000000000,    22312.0000000000,  93355.0000000000,  85898.0000000000,  -5449.0000000000,  1980.0000000000,   22497.0000000000,
    92044.0000000000,  85898.0000000000,  -5634.0000000000,  3693.0000000000,   23070.0000000000,  88136.0000000000,  85898.0000000000,
    -6175.0000000000,  2376.0000000000,   21541.0000000000,  99069.0000000000,  -23.0000000000,    4499.0000000000,   867.0000000000,
    21161.0000000000,  102113.0000000000, -23.0000000000,    4043.0000000000,   -704.0000000000,   21038.0000000000,  103135.0000000000,
    -23.0000000000,    3887.0000000000,   -2268.0000000000,  21161.0000000000,  102113.0000000000, -23.0000000000,    4043.0000000000,
    -3753.0000000000,  21540.0000000000,  99070.0000000000,  -23.0000000000,    4500.0000000000,   -3320.0000000000,  23523.0000000000,
    85155.0000000000,  -23.0000000000,    6456.0000000000,   -2015.0000000000,  23067.0000000000,  88158.0000000000,  -23.0000000000,
    6049.0000000000,   -657.0000000000,   22918.0000000000,  89157.0000000000,  -23.0000000000,    5912.0000000000,   704.0000000000,
    23067.0000000000,  88159.0000000000,  -23.0000000000,    6049.0000000000,   2025.0000000000,   23523.0000000000,  85154.0000000000,
    -23.0000000000,    6456.0000000000,   1753.0000000000,   26201.0000000000,  68787.0000000000,  -23.0000000000,    8574.0000000000,
    580.0000000000,    25657.0000000000,  71976.0000000000,  -23.0000000000,    8173.0000000000,   -622.0000000000,   25481.0000000000,
    73024.0000000000,  -23.0000000000,    8040.0000000000,   -1820.0000000000,  25658.0000000000,  71975.0000000000,  -23.0000000000,
    8173.0000000000,   -2983.0000000000,  26201.0000000000,  68789.0000000000,  -23.0000000000,    8574.0000000000};

#include "Joint_curvea.cpp" //Introduce a array
#include "Joint_curveb.cpp" //Introduce b array
#include "Joint_curvec.cpp" //Introduce c array

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
inline void Set_current_XYZE(const float (&Set_current_XYZ_point)[XYZE])
{
  current_position[X_AXIS] = Set_current_XYZ_point[X_AXIS];
  current_position[Y_AXIS] = Set_current_XYZ_point[Y_AXIS];
  current_position[Z_AXIS] = Set_current_XYZ_point[Z_AXIS];
  current_position[E_AXIS] = Set_current_XYZ_point[E_AXIS];

  DEBUG_POS("Set_current", current_position);
}

inline void Set_current_Joint(const int32_t (&Set_current_Joint_point)[Joint_All])
{
  current_position_Joint[Joint1_AXIS] = Set_current_Joint_point[Joint1_AXIS];
  current_position_Joint[Joint2_AXIS] = Set_current_Joint_point[Joint2_AXIS];
  current_position_Joint[Joint3_AXIS] = Set_current_Joint_point[Joint3_AXIS];
  current_position_Joint[Joint4_AXIS] = Set_current_Joint_point[Joint4_AXIS];
  current_position_Joint[Joint5_AXIS] = Set_current_Joint_point[Joint5_AXIS];

  DEBUG_POS_Joint("Set_current_Joint", current_position_Joint);
}

inline void Set_current_Joint_5(const int32_t Set_current_Joint_point1, const int32_t Set_current_Joint_point2,
                                const int32_t Set_current_Joint_point3, const int32_t Set_current_Joint_point4,
                                const int32_t Set_current_Joint_point5)
{
  current_position_Joint[Joint1_AXIS] = Set_current_Joint_point1;
  current_position_Joint[Joint2_AXIS] = Set_current_Joint_point2;
  current_position_Joint[Joint3_AXIS] = Set_current_Joint_point3;
  current_position_Joint[Joint4_AXIS] = Set_current_Joint_point4;
  current_position_Joint[Joint5_AXIS] = Set_current_Joint_point5;

  DEBUG_POS_Joint("Set_current_Joint_5", current_position_Joint);
}

inline void Set_current_Joint_Curve(const float point)
{
  DEBUG_POS_Joint("(Before)Set_current_Joint_Curve", current_position_Joint);
  float point1 = point * 100;
  current_position_Joint[Joint1_AXIS] = a[0] * point1 * point1 + b[0] * point1 + c[0];
  current_position_Joint[Joint2_AXIS] = a[1] * point1 * point1 + b[1] * point1 + c[1];
  current_position_Joint[Joint3_AXIS] = a[2] * point1 * point1 + b[2] * point1 + c[2];
  current_position_Joint[Joint4_AXIS] = a[3] * point1 * point1 + b[3] * point1 + c[3];
  current_position_Joint[Joint5_AXIS] = a[4] * point1 * point1 + b[4] * point1 + c[4];

  // SERIAL_ECHOLNPAIR(" point:", point);

  DEBUG_POS_Joint("(After)Set_current_Joint_Curve", current_position_Joint);
}

inline void Set_current_Joint_Curve_many(const int num_total, const float point)
{
  DEBUG_POS_Joint("(Before)Set_current_Joint_Curve_many", current_position_Joint);
  float point1 = point * 100;
  current_position_Joint[Joint1_AXIS] = pgm_read_float_near(&a_m2[num_total * 5 + 0]) * point1 * point1 +
                                        pgm_read_float_near(&b_m2[num_total * 5 + 0]) * point1 + pgm_read_float_near(&c_m2[num_total * 5 + 0]);
  current_position_Joint[Joint2_AXIS] = pgm_read_float_near(&a_m2[num_total * 5 + 1]) * point1 * point1 +
                                        pgm_read_float_near(&b_m2[num_total * 5 + 1]) * point1 + pgm_read_float_near(&c_m2[num_total * 5 + 1]);
  current_position_Joint[Joint3_AXIS] = pgm_read_float_near(&a_m2[num_total * 5 + 2]) * point1 * point1 +
                                        pgm_read_float_near(&b_m2[num_total * 5 + 2]) * point1 + pgm_read_float_near(&c_m2[num_total * 5 + 2]);
  current_position_Joint[Joint4_AXIS] = pgm_read_float_near(&a_m2[num_total * 5 + 3]) * point1 * point1 +
                                        pgm_read_float_near(&b_m2[num_total * 5 + 3]) * point1 + pgm_read_float_near(&c_m2[num_total * 5 + 3]);
  current_position_Joint[Joint5_AXIS] = pgm_read_float_near(&a_m2[num_total * 5 + 4]) * point1 * point1 +
                                        pgm_read_float_near(&b_m2[num_total * 5 + 4]) * point1 + pgm_read_float_near(&c_m2[num_total * 5 + 4]);

  // SERIAL_ECHOLNPAIR(" point:", point);

  DEBUG_POS_Joint("(After)Set_current_Joint_Curve_many", current_position_Joint);
}

int X_Y_to_Number(const float Nx, const float Ny)
{
  int tempX = (int)(Nx / 45);
  int tempY = (int)(Ny / 45);
  int temp = 0;
  // SERIAL_ECHOLNPAIR("tempY*5+tempX:", tempY*5+tempX);
  switch (tempY * 5 + tempX)
  { // y*5+x
    case 0: temp = 0; break;
    case 1: temp = 1; break;
    case 2: temp = 2; break;
    case 3: temp = 3; break;
    case 4: temp = 4; break;
    case 9: temp = 5; break;
    case 8: temp = 6; break;
    case 7: temp = 7; break;
    case 6: temp = 8; break;
    case 5: temp = 9; break;
    case 10: temp = 10; break;
    case 11: temp = 11; break;
    case 12: temp = 12; break;
    case 13: temp = 13; break;
    case 14: temp = 14; break;
    case 19: temp = 15; break;
    case 18: temp = 16; break;
    case 17: temp = 17; break;
    case 16: temp = 18; break;
    case 15: temp = 19; break;
    case 20: temp = 20; break;
    case 21: temp = 21; break;
    case 22: temp = 22; break;
    case 23: temp = 23; break;
    case 24: temp = 24; break;
  }
  return temp;
}

float Forward_Curve(float point_FC, int number_mat, int Joint_num)
{
  // SERIAL_ECHOPAIR("point_FC:", point_FC);
  // SERIAL_ECHOPAIR(" number_mat:", number_mat);
  // SERIAL_ECHOPAIR(" Joint_num:", Joint_num);
  // float temp_return = c_m[number_mat][Joint_num];
  float temp_return = pgm_read_float_near(&a_m1[number_mat * 5 + Joint_num]) * point_FC * point_FC +
                      pgm_read_float_near(&b_m1[number_mat * 5 + Joint_num]) * point_FC +
                      pgm_read_float_near(&c_m1[number_mat * 5 + Joint_num]); // + b_m[0][Joint_num]*point_FC + c_m[0][Joint_num];
  // SERIAL_ECHOPAIR(" return data:", temp_return);
  // SERIAL_ECHOLNPAIR(" c_m[0][0]:", c_m[0][Joint_num]);

  return temp_return;
}

void Set_current_Joint_Curve_More(float numberx, float numbery, float numberz)
{
  int number = X_Y_to_Number(numberx, numbery);
  // int number = 0;
  // SERIAL_ECHOLNPAIR("number:", number);

  // DEBUG_POS_Joint("(Before)Set_current_Joint_Curve_More", current_position_Joint);
  float point1 = numberz * 100;

  // SERIAL_ECHOLNPAIR("Forward_Curve(2000,0,0)", Forward_Curve(2000,0,0));

  current_position_Joint[Joint1_AXIS] = (int32_t)Forward_Curve(point1, number, Joint1_AXIS);
  current_position_Joint[Joint2_AXIS] = (int32_t)Forward_Curve(point1, number, Joint2_AXIS);
  current_position_Joint[Joint3_AXIS] = (int32_t)Forward_Curve(point1, number, Joint3_AXIS);
  current_position_Joint[Joint4_AXIS] = (int32_t)Forward_Curve(point1, number, Joint4_AXIS);
  current_position_Joint[Joint5_AXIS] = (int32_t)Forward_Curve(point1, number, Joint5_AXIS);

  // SERIAL_ECHOPAIR(" point:", point);
  // SERIAL_ECHOPAIR(" x:", numberx);
  // SERIAL_ECHOPAIR("-y:", numbery);

  // DEBUG_POS_Joint("(After)Set_current_Joint_Curve_More", current_position_Joint);
}

inline void Set_current_Joint_Slope(const int32_t (&Set_current_Joint_data)[Joint_All], const float (&Set_current_Joint_slope)[Joint_All],
                                    const float point)
{
  DEBUG_POS_Joint("(Before)Set_current_Joint_Slope", Set_current_Joint_data);

  current_position_Joint[Joint1_AXIS] = Set_current_Joint_data[Joint1_AXIS] + (int32_t)LROUND(Set_current_Joint_slope[Joint1_AXIS] * point);
  current_position_Joint[Joint2_AXIS] = Set_current_Joint_data[Joint2_AXIS] + (int32_t)LROUND(Set_current_Joint_slope[Joint2_AXIS] * point);
  current_position_Joint[Joint3_AXIS] = Set_current_Joint_data[Joint3_AXIS] + (int32_t)LROUND(Set_current_Joint_slope[Joint3_AXIS] * point);
  current_position_Joint[Joint4_AXIS] = Set_current_Joint_data[Joint4_AXIS] + (int32_t)LROUND(Set_current_Joint_slope[Joint4_AXIS] * point);
  current_position_Joint[Joint5_AXIS] = Set_current_Joint_data[Joint5_AXIS] + (int32_t)LROUND(Set_current_Joint_slope[Joint5_AXIS] * point);

  SERIAL_ECHOPAIR("Slope J:", Set_current_Joint_slope[Joint1_AXIS]);
  SERIAL_ECHOPAIR(" Slope A:", Set_current_Joint_slope[Joint2_AXIS]);
  SERIAL_ECHOPAIR(" Slope B:", Set_current_Joint_slope[Joint3_AXIS]);
  SERIAL_ECHOPAIR(" Slope C:", Set_current_Joint_slope[Joint4_AXIS]);
  SERIAL_ECHOPAIR(" Slope D:", Set_current_Joint_slope[Joint5_AXIS]);

  SERIAL_ECHOLNPAIR(" point:", point);

  DEBUG_POS_Joint("(After)Set_current_Joint_Slope", current_position_Joint);
}

inline float Delta_Z_01mm(const float data1, const float data2) { return (data1 - data2) * 100; }

inline void buffer_line_to_destination_Constant(const float (&Set_Position)[XYZE], const int32_t (&Set_Position_Joint)[Joint_All],
                                                const int32_t &fr_mm_s)
{
#if ENABLED(HANGPRINTER)
  UNUSED(fr_mm_s);
#else
  // SERIAL_ECHOLNPGM("<<< buffer_line_to_destination_Constant");

  current_position[X_AXIS] = Set_Position[X_AXIS];
  current_position[Y_AXIS] = Set_Position[Y_AXIS];
  current_position[Z_AXIS] = Set_Position[Z_AXIS];
  current_position[E_AXIS] = Set_Position[E_AXIS];

  current_position_Joint[Joint1_AXIS] = Set_Position_Joint[Joint1_AXIS];
  current_position_Joint[Joint2_AXIS] = Set_Position_Joint[Joint2_AXIS];
  current_position_Joint[Joint3_AXIS] = Set_Position_Joint[Joint3_AXIS];
  current_position_Joint[Joint4_AXIS] = Set_Position_Joint[Joint4_AXIS];
  current_position_Joint[Joint5_AXIS] = Set_Position_Joint[Joint5_AXIS];

  planner.buffer_line_joint(Set_Position[X_AXIS], Set_Position[Y_AXIS], Set_Position[Z_AXIS], Set_Position_Joint[Joint1_AXIS],
                            Set_Position_Joint[Joint2_AXIS], Set_Position_Joint[Joint3_AXIS], Set_Position_Joint[Joint4_AXIS],
                            Set_Position_Joint[Joint5_AXIS], Set_Position[E_CART], fr_mm_s, active_extruder);
  // SERIAL_ECHOLNPAIR("feedrate_mm_s:",fr_mm_s);
  sync_plan_position();

// SERIAL_ECHOLNPGM(">>> buffer_line_to_destination_Constant");
#endif
}

inline void buffer_line_to_destination(const int32_t &fr_mm_s)
{
#if ENABLED(HANGPRINTER)
  UNUSED(fr_mm_s);
#else
  planner.buffer_line_joint(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination_Joint[Joint1_AXIS],
                            destination_Joint[Joint2_AXIS], destination_Joint[Joint3_AXIS], destination_Joint[Joint4_AXIS],
                            destination_Joint[Joint5_AXIS], destination[E_CART], fr_mm_s, active_extruder);
// SERIAL_ECHOLNPAIR("feedrate_mm_s:",fr_mm_s);
#endif
}

bool circle_inside_d(float cir_x, float cir_y)
{
  if (pow((cir_x - Circle_Center_X), 2) + pow((cir_y - Circle_Center_Y), 2) <= pow(Circle_outside_r + 1, 2)) return 1;
  else
    return 0;
}

bool circle_outside_d(float cor_x, float cor_y)
{
  if (pow((cor_x - Circle_Center_X), 2) + pow((cor_y - Circle_Center_Y), 2) >= pow(Circle_inside_r - 1, 2)) return 1;
  else
    return 0;
}

bool find_region_in_out_d(float x_fr, float y_fr)
{
  if ((circle_inside_d(x_fr, y_fr) && circle_outside_d(x_fr, y_fr)) == 1)
  {
    SERIAL_ECHOLNPGM("In the region");
    return 1;
  }
  else
  {
    SERIAL_ECHOLNPGM("Out of region");
    return 0;
  }
}

bool In_Rectangle(float IR_X, float IR_Y)
{
  SERIAL_ECHOPAIR("In_Rectangle X:", IR_X);
  SERIAL_ECHOPAIR(" Y:", IR_Y);
  if ((IR_X >= 209.837014 && IR_X <= 670.16299) && ((IR_Y >= 615 && IR_Y <= 815)))
  {
    SERIAL_ECHOLNPGM(" Within a rectangular measuring point");
    return 1;
  }
  else
  {
    SERIAL_ECHOLNPGM(" Not within a rectangular measuring point");
    return 0;
  }
}

int Use_XY_to_Matrix_Index(float UXYMIX, float UXYMIY)
{
#if ENABLED(DEBUG_LEVELING_FEATURE)
  SERIAL_ECHOLNPGM(">>> Use_XY_to_Matrix_Index");
#endif
  int temp_UXYMIX = (int)UXYMIX;
  int temp_UXYMIY = (int)UXYMIY;

  if (UXYMIX == 209.837014) temp_UXYMIX = 209;
  else if (UXYMIX == 670.16299)
    temp_UXYMIX = 670;

  SERIAL_ECHOPAIR("temp_UXYMIX: ", temp_UXYMIX);
  SERIAL_ECHOLNPAIR(" temp_UXYMIY: ", temp_UXYMIY);

  int temp_return = 0;

  switch (temp_UXYMIX)
  {
    case 210: // 209:
      switch (temp_UXYMIY)
      {
        case 615: temp_return = 62; break;
        case 715: temp_return = 72; break;
        case 815: temp_return = 82; break;
      }
      break;
    case 300:
      switch (temp_UXYMIY)
      {
        case 615: temp_return = 63; break;
        case 715: temp_return = 73; break;
        case 815: temp_return = 83; break;
      }
      break;
    case 400:
      switch (temp_UXYMIY)
      {
        case 615: temp_return = 64; break;
        case 715: temp_return = 74; break;
        case 815: temp_return = 84; break;
      }
      break;
    case 500:
      switch (temp_UXYMIY)
      {
        case 615: temp_return = 65; break;
        case 715: temp_return = 75; break;
        case 815: temp_return = 85; break;
      }
      break;
    case 600:
      switch (temp_UXYMIY)
      {
        case 615: temp_return = 66; break;
        case 715: temp_return = 76; break;
        case 815: temp_return = 86; break;
      }
      break;
    case 670:
      switch (temp_UXYMIY)
      {
        case 615: temp_return = 67; break;
        case 715: temp_return = 77; break;
        case 815: temp_return = 87; break;
      }
      break;
  }
#if ENABLED(DEBUG_LEVELING_FEATURE)
  SERIAL_ECHOLNPAIR("Return Value: ", temp_return);
  SERIAL_ECHOLNPGM("<<< Use_XY_to_Matrix_Index");
#endif
  return temp_return;
}

static float Reverse_Curve(const int32_t temp_pos, JointEnum Joint)
{
  float temp_return = 0;

#if ENABLED(DEBUG_LEVELING_FEATURE)
  if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(">>> Reverse_Curve");
#endif

  SERIAL_ECHOLNPAIR("  Position Joint: ", temp_pos);

  float ba2 = b[Joint] / a[Joint] / 2;
  float temp1 = (temp_pos - c[Joint]) / a[Joint]; //
  float temp2 = ba2 * ba2;                        // X
  float temp3[2] = {0};
  temp3[1] = temp2 / 1000000;
  temp3[0] = (int32_t)(temp2) % 1000000;

  SERIAL_ECHOLNPAIR("  a: ", a[Joint]);
  SERIAL_ECHOLNPAIR("  b: ", b[Joint]);
  SERIAL_ECHOLNPAIR("  c: ", c[Joint]);
  SERIAL_ECHOLNPAIR("  b/a/2: ", ba2);
  SERIAL_ECHOLNPAIR("  (J-c)/a: ", temp1);
  SERIAL_ECHOLNPAIR("  [(b/a)/2]^2: ", temp2);
  SERIAL_ECHOLNPAIR("  [(b/a)/2]^2(*10^6): ", temp3[1]); // Show Data
  SERIAL_ECHOLNPAIR("  [(b/a)/2]^2(*10^0): ", temp3[0]);

  temp_return = (float)(-sqrt(temp1 + temp2) - ba2);
  if (temp_return <= -3000) temp_return = (float)(+sqrt(temp1 + temp2) - ba2);
  // temp_return=sqrt((temp_pos-c[Joint])/a[Joint]+ pow((b[Joint]/a[Joint])/2,2))-(b[Joint]/a[Joint])/2;
  SERIAL_ECHOLNPAIR("  temp_return: ", temp_return);

#if ENABLED(DEBUG_LEVELING_FEATURE)
  if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< Reverse_Curve");
#endif

  return temp_return;
}

static float Reverse_Curve_Many(const int num_total, const int32_t temp_pos, JointEnum Joint)
{
  float temp_return = 0;

#if ENABLED(DEBUG_LEVELING_FEATURE)
  if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(">>> Reverse_Curve_Many");
#endif

  SERIAL_ECHOLNPAIR("  Position Joint: ", temp_pos);
  SERIAL_ECHOLNPAIR("  Num_total: ", num_total);

  float ba2 = pgm_read_float_near(&b_m2[num_total * 5 + Joint]) / pgm_read_float_near(&a_m2[num_total * 5 + Joint]) / 2;
  float temp1 = (temp_pos - pgm_read_float_near(&c_m2[num_total * 5 + Joint])) / pgm_read_float_near(&a_m2[num_total * 5 + Joint]); //
  float temp2 = ba2 * ba2;                                                                                                          // X
  float temp3[2] = {0};
  temp3[1] = temp2 / 100000;
  temp3[0] = (int32_t)(temp2) % 100000;

  SERIAL_ECHOLNPAIR("  a: ", pgm_read_float_near(&a_m2[num_total * 5 + Joint]));
  SERIAL_ECHOLNPAIR("  b: ", pgm_read_float_near(&b_m2[num_total * 5 + Joint]));
  SERIAL_ECHOLNPAIR("  c: ", pgm_read_float_near(&c_m2[num_total * 5 + Joint]));
  SERIAL_ECHOLNPAIR("  b/a/2: ", ba2);
  SERIAL_ECHOLNPAIR("  (J-c)/a: ", temp1);
  SERIAL_ECHOLNPAIR("  [(b/a)/2]^2: ", temp2);
  SERIAL_ECHOLNPAIR("  [(b/a)/2]^2(*10^6): ", temp3[1]); // Show Data
  SERIAL_ECHOLNPAIR("  [(b/a)/2]^2(*10^0): ", temp3[0]);

  temp_return = (float)(-sqrt(temp1 + temp2) - ba2);
  if (temp_return <= -3000) temp_return = (float)(+sqrt(temp1 + temp2) - ba2);
  // temp_return=sqrt((temp_pos-c[Joint])/a[Joint]+ pow((b[Joint]/a[Joint])/2,2))-(b[Joint]/a[Joint])/2;
  SERIAL_ECHOLNPAIR("  temp_return: ", temp_return);

#if ENABLED(DEBUG_LEVELING_FEATURE)
  if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< Reverse_Curve_Many");
#endif

  return temp_return;
}