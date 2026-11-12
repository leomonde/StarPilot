#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_188828970539226358) {
   out_188828970539226358[0] = delta_x[0] + nom_x[0];
   out_188828970539226358[1] = delta_x[1] + nom_x[1];
   out_188828970539226358[2] = delta_x[2] + nom_x[2];
   out_188828970539226358[3] = delta_x[3] + nom_x[3];
   out_188828970539226358[4] = delta_x[4] + nom_x[4];
   out_188828970539226358[5] = delta_x[5] + nom_x[5];
   out_188828970539226358[6] = delta_x[6] + nom_x[6];
   out_188828970539226358[7] = delta_x[7] + nom_x[7];
   out_188828970539226358[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5608983488306314815) {
   out_5608983488306314815[0] = -nom_x[0] + true_x[0];
   out_5608983488306314815[1] = -nom_x[1] + true_x[1];
   out_5608983488306314815[2] = -nom_x[2] + true_x[2];
   out_5608983488306314815[3] = -nom_x[3] + true_x[3];
   out_5608983488306314815[4] = -nom_x[4] + true_x[4];
   out_5608983488306314815[5] = -nom_x[5] + true_x[5];
   out_5608983488306314815[6] = -nom_x[6] + true_x[6];
   out_5608983488306314815[7] = -nom_x[7] + true_x[7];
   out_5608983488306314815[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8344546280501347360) {
   out_8344546280501347360[0] = 1.0;
   out_8344546280501347360[1] = 0.0;
   out_8344546280501347360[2] = 0.0;
   out_8344546280501347360[3] = 0.0;
   out_8344546280501347360[4] = 0.0;
   out_8344546280501347360[5] = 0.0;
   out_8344546280501347360[6] = 0.0;
   out_8344546280501347360[7] = 0.0;
   out_8344546280501347360[8] = 0.0;
   out_8344546280501347360[9] = 0.0;
   out_8344546280501347360[10] = 1.0;
   out_8344546280501347360[11] = 0.0;
   out_8344546280501347360[12] = 0.0;
   out_8344546280501347360[13] = 0.0;
   out_8344546280501347360[14] = 0.0;
   out_8344546280501347360[15] = 0.0;
   out_8344546280501347360[16] = 0.0;
   out_8344546280501347360[17] = 0.0;
   out_8344546280501347360[18] = 0.0;
   out_8344546280501347360[19] = 0.0;
   out_8344546280501347360[20] = 1.0;
   out_8344546280501347360[21] = 0.0;
   out_8344546280501347360[22] = 0.0;
   out_8344546280501347360[23] = 0.0;
   out_8344546280501347360[24] = 0.0;
   out_8344546280501347360[25] = 0.0;
   out_8344546280501347360[26] = 0.0;
   out_8344546280501347360[27] = 0.0;
   out_8344546280501347360[28] = 0.0;
   out_8344546280501347360[29] = 0.0;
   out_8344546280501347360[30] = 1.0;
   out_8344546280501347360[31] = 0.0;
   out_8344546280501347360[32] = 0.0;
   out_8344546280501347360[33] = 0.0;
   out_8344546280501347360[34] = 0.0;
   out_8344546280501347360[35] = 0.0;
   out_8344546280501347360[36] = 0.0;
   out_8344546280501347360[37] = 0.0;
   out_8344546280501347360[38] = 0.0;
   out_8344546280501347360[39] = 0.0;
   out_8344546280501347360[40] = 1.0;
   out_8344546280501347360[41] = 0.0;
   out_8344546280501347360[42] = 0.0;
   out_8344546280501347360[43] = 0.0;
   out_8344546280501347360[44] = 0.0;
   out_8344546280501347360[45] = 0.0;
   out_8344546280501347360[46] = 0.0;
   out_8344546280501347360[47] = 0.0;
   out_8344546280501347360[48] = 0.0;
   out_8344546280501347360[49] = 0.0;
   out_8344546280501347360[50] = 1.0;
   out_8344546280501347360[51] = 0.0;
   out_8344546280501347360[52] = 0.0;
   out_8344546280501347360[53] = 0.0;
   out_8344546280501347360[54] = 0.0;
   out_8344546280501347360[55] = 0.0;
   out_8344546280501347360[56] = 0.0;
   out_8344546280501347360[57] = 0.0;
   out_8344546280501347360[58] = 0.0;
   out_8344546280501347360[59] = 0.0;
   out_8344546280501347360[60] = 1.0;
   out_8344546280501347360[61] = 0.0;
   out_8344546280501347360[62] = 0.0;
   out_8344546280501347360[63] = 0.0;
   out_8344546280501347360[64] = 0.0;
   out_8344546280501347360[65] = 0.0;
   out_8344546280501347360[66] = 0.0;
   out_8344546280501347360[67] = 0.0;
   out_8344546280501347360[68] = 0.0;
   out_8344546280501347360[69] = 0.0;
   out_8344546280501347360[70] = 1.0;
   out_8344546280501347360[71] = 0.0;
   out_8344546280501347360[72] = 0.0;
   out_8344546280501347360[73] = 0.0;
   out_8344546280501347360[74] = 0.0;
   out_8344546280501347360[75] = 0.0;
   out_8344546280501347360[76] = 0.0;
   out_8344546280501347360[77] = 0.0;
   out_8344546280501347360[78] = 0.0;
   out_8344546280501347360[79] = 0.0;
   out_8344546280501347360[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1981076499881668902) {
   out_1981076499881668902[0] = state[0];
   out_1981076499881668902[1] = state[1];
   out_1981076499881668902[2] = state[2];
   out_1981076499881668902[3] = state[3];
   out_1981076499881668902[4] = state[4];
   out_1981076499881668902[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1981076499881668902[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1981076499881668902[7] = state[7];
   out_1981076499881668902[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3506768513775075149) {
   out_3506768513775075149[0] = 1;
   out_3506768513775075149[1] = 0;
   out_3506768513775075149[2] = 0;
   out_3506768513775075149[3] = 0;
   out_3506768513775075149[4] = 0;
   out_3506768513775075149[5] = 0;
   out_3506768513775075149[6] = 0;
   out_3506768513775075149[7] = 0;
   out_3506768513775075149[8] = 0;
   out_3506768513775075149[9] = 0;
   out_3506768513775075149[10] = 1;
   out_3506768513775075149[11] = 0;
   out_3506768513775075149[12] = 0;
   out_3506768513775075149[13] = 0;
   out_3506768513775075149[14] = 0;
   out_3506768513775075149[15] = 0;
   out_3506768513775075149[16] = 0;
   out_3506768513775075149[17] = 0;
   out_3506768513775075149[18] = 0;
   out_3506768513775075149[19] = 0;
   out_3506768513775075149[20] = 1;
   out_3506768513775075149[21] = 0;
   out_3506768513775075149[22] = 0;
   out_3506768513775075149[23] = 0;
   out_3506768513775075149[24] = 0;
   out_3506768513775075149[25] = 0;
   out_3506768513775075149[26] = 0;
   out_3506768513775075149[27] = 0;
   out_3506768513775075149[28] = 0;
   out_3506768513775075149[29] = 0;
   out_3506768513775075149[30] = 1;
   out_3506768513775075149[31] = 0;
   out_3506768513775075149[32] = 0;
   out_3506768513775075149[33] = 0;
   out_3506768513775075149[34] = 0;
   out_3506768513775075149[35] = 0;
   out_3506768513775075149[36] = 0;
   out_3506768513775075149[37] = 0;
   out_3506768513775075149[38] = 0;
   out_3506768513775075149[39] = 0;
   out_3506768513775075149[40] = 1;
   out_3506768513775075149[41] = 0;
   out_3506768513775075149[42] = 0;
   out_3506768513775075149[43] = 0;
   out_3506768513775075149[44] = 0;
   out_3506768513775075149[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3506768513775075149[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3506768513775075149[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3506768513775075149[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3506768513775075149[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3506768513775075149[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3506768513775075149[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3506768513775075149[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3506768513775075149[53] = -9.8100000000000005*dt;
   out_3506768513775075149[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3506768513775075149[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3506768513775075149[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3506768513775075149[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3506768513775075149[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3506768513775075149[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3506768513775075149[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3506768513775075149[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3506768513775075149[62] = 0;
   out_3506768513775075149[63] = 0;
   out_3506768513775075149[64] = 0;
   out_3506768513775075149[65] = 0;
   out_3506768513775075149[66] = 0;
   out_3506768513775075149[67] = 0;
   out_3506768513775075149[68] = 0;
   out_3506768513775075149[69] = 0;
   out_3506768513775075149[70] = 1;
   out_3506768513775075149[71] = 0;
   out_3506768513775075149[72] = 0;
   out_3506768513775075149[73] = 0;
   out_3506768513775075149[74] = 0;
   out_3506768513775075149[75] = 0;
   out_3506768513775075149[76] = 0;
   out_3506768513775075149[77] = 0;
   out_3506768513775075149[78] = 0;
   out_3506768513775075149[79] = 0;
   out_3506768513775075149[80] = 1;
}
void h_25(double *state, double *unused, double *out_903869287523985863) {
   out_903869287523985863[0] = state[6];
}
void H_25(double *state, double *unused, double *out_423208247527487555) {
   out_423208247527487555[0] = 0;
   out_423208247527487555[1] = 0;
   out_423208247527487555[2] = 0;
   out_423208247527487555[3] = 0;
   out_423208247527487555[4] = 0;
   out_423208247527487555[5] = 0;
   out_423208247527487555[6] = 1;
   out_423208247527487555[7] = 0;
   out_423208247527487555[8] = 0;
}
void h_24(double *state, double *unused, double *out_3835652701292500148) {
   out_3835652701292500148[0] = state[4];
   out_3835652701292500148[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3134717351274835581) {
   out_3134717351274835581[0] = 0;
   out_3134717351274835581[1] = 0;
   out_3134717351274835581[2] = 0;
   out_3134717351274835581[3] = 0;
   out_3134717351274835581[4] = 1;
   out_3134717351274835581[5] = 0;
   out_3134717351274835581[6] = 0;
   out_3134717351274835581[7] = 0;
   out_3134717351274835581[8] = 0;
   out_3134717351274835581[9] = 0;
   out_3134717351274835581[10] = 0;
   out_3134717351274835581[11] = 0;
   out_3134717351274835581[12] = 0;
   out_3134717351274835581[13] = 0;
   out_3134717351274835581[14] = 1;
   out_3134717351274835581[15] = 0;
   out_3134717351274835581[16] = 0;
   out_3134717351274835581[17] = 0;
}
void h_30(double *state, double *unused, double *out_8667134544344658304) {
   out_8667134544344658304[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2095124710979761072) {
   out_2095124710979761072[0] = 0;
   out_2095124710979761072[1] = 0;
   out_2095124710979761072[2] = 0;
   out_2095124710979761072[3] = 0;
   out_2095124710979761072[4] = 1;
   out_2095124710979761072[5] = 0;
   out_2095124710979761072[6] = 0;
   out_2095124710979761072[7] = 0;
   out_2095124710979761072[8] = 0;
}
void h_26(double *state, double *unused, double *out_5630348492900325619) {
   out_5630348492900325619[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4164711566401543779) {
   out_4164711566401543779[0] = 0;
   out_4164711566401543779[1] = 0;
   out_4164711566401543779[2] = 0;
   out_4164711566401543779[3] = 0;
   out_4164711566401543779[4] = 0;
   out_4164711566401543779[5] = 0;
   out_4164711566401543779[6] = 0;
   out_4164711566401543779[7] = 1;
   out_4164711566401543779[8] = 0;
}
void h_27(double *state, double *unused, double *out_1847379140425648298) {
   out_1847379140425648298[0] = state[3];
}
void H_27(double *state, double *unused, double *out_79638600820663839) {
   out_79638600820663839[0] = 0;
   out_79638600820663839[1] = 0;
   out_79638600820663839[2] = 0;
   out_79638600820663839[3] = 1;
   out_79638600820663839[4] = 0;
   out_79638600820663839[5] = 0;
   out_79638600820663839[6] = 0;
   out_79638600820663839[7] = 0;
   out_79638600820663839[8] = 0;
}
void h_29(double *state, double *unused, double *out_3665461276683812659) {
   out_3665461276683812659[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2605356055294153256) {
   out_2605356055294153256[0] = 0;
   out_2605356055294153256[1] = 1;
   out_2605356055294153256[2] = 0;
   out_2605356055294153256[3] = 0;
   out_2605356055294153256[4] = 0;
   out_2605356055294153256[5] = 0;
   out_2605356055294153256[6] = 0;
   out_2605356055294153256[7] = 0;
   out_2605356055294153256[8] = 0;
}
void h_28(double *state, double *unused, double *out_9164221498855202505) {
   out_9164221498855202505[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2477042961775377318) {
   out_2477042961775377318[0] = 1;
   out_2477042961775377318[1] = 0;
   out_2477042961775377318[2] = 0;
   out_2477042961775377318[3] = 0;
   out_2477042961775377318[4] = 0;
   out_2477042961775377318[5] = 0;
   out_2477042961775377318[6] = 0;
   out_2477042961775377318[7] = 0;
   out_2477042961775377318[8] = 0;
}
void h_31(double *state, double *unused, double *out_6556689476724902462) {
   out_6556689476724902462[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4790919668634895255) {
   out_4790919668634895255[0] = 0;
   out_4790919668634895255[1] = 0;
   out_4790919668634895255[2] = 0;
   out_4790919668634895255[3] = 0;
   out_4790919668634895255[4] = 0;
   out_4790919668634895255[5] = 0;
   out_4790919668634895255[6] = 0;
   out_4790919668634895255[7] = 0;
   out_4790919668634895255[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_188828970539226358) {
  err_fun(nom_x, delta_x, out_188828970539226358);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5608983488306314815) {
  inv_err_fun(nom_x, true_x, out_5608983488306314815);
}
void car_H_mod_fun(double *state, double *out_8344546280501347360) {
  H_mod_fun(state, out_8344546280501347360);
}
void car_f_fun(double *state, double dt, double *out_1981076499881668902) {
  f_fun(state,  dt, out_1981076499881668902);
}
void car_F_fun(double *state, double dt, double *out_3506768513775075149) {
  F_fun(state,  dt, out_3506768513775075149);
}
void car_h_25(double *state, double *unused, double *out_903869287523985863) {
  h_25(state, unused, out_903869287523985863);
}
void car_H_25(double *state, double *unused, double *out_423208247527487555) {
  H_25(state, unused, out_423208247527487555);
}
void car_h_24(double *state, double *unused, double *out_3835652701292500148) {
  h_24(state, unused, out_3835652701292500148);
}
void car_H_24(double *state, double *unused, double *out_3134717351274835581) {
  H_24(state, unused, out_3134717351274835581);
}
void car_h_30(double *state, double *unused, double *out_8667134544344658304) {
  h_30(state, unused, out_8667134544344658304);
}
void car_H_30(double *state, double *unused, double *out_2095124710979761072) {
  H_30(state, unused, out_2095124710979761072);
}
void car_h_26(double *state, double *unused, double *out_5630348492900325619) {
  h_26(state, unused, out_5630348492900325619);
}
void car_H_26(double *state, double *unused, double *out_4164711566401543779) {
  H_26(state, unused, out_4164711566401543779);
}
void car_h_27(double *state, double *unused, double *out_1847379140425648298) {
  h_27(state, unused, out_1847379140425648298);
}
void car_H_27(double *state, double *unused, double *out_79638600820663839) {
  H_27(state, unused, out_79638600820663839);
}
void car_h_29(double *state, double *unused, double *out_3665461276683812659) {
  h_29(state, unused, out_3665461276683812659);
}
void car_H_29(double *state, double *unused, double *out_2605356055294153256) {
  H_29(state, unused, out_2605356055294153256);
}
void car_h_28(double *state, double *unused, double *out_9164221498855202505) {
  h_28(state, unused, out_9164221498855202505);
}
void car_H_28(double *state, double *unused, double *out_2477042961775377318) {
  H_28(state, unused, out_2477042961775377318);
}
void car_h_31(double *state, double *unused, double *out_6556689476724902462) {
  h_31(state, unused, out_6556689476724902462);
}
void car_H_31(double *state, double *unused, double *out_4790919668634895255) {
  H_31(state, unused, out_4790919668634895255);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
