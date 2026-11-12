#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_188828970539226358);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5608983488306314815);
void car_H_mod_fun(double *state, double *out_8344546280501347360);
void car_f_fun(double *state, double dt, double *out_1981076499881668902);
void car_F_fun(double *state, double dt, double *out_3506768513775075149);
void car_h_25(double *state, double *unused, double *out_903869287523985863);
void car_H_25(double *state, double *unused, double *out_423208247527487555);
void car_h_24(double *state, double *unused, double *out_3835652701292500148);
void car_H_24(double *state, double *unused, double *out_3134717351274835581);
void car_h_30(double *state, double *unused, double *out_8667134544344658304);
void car_H_30(double *state, double *unused, double *out_2095124710979761072);
void car_h_26(double *state, double *unused, double *out_5630348492900325619);
void car_H_26(double *state, double *unused, double *out_4164711566401543779);
void car_h_27(double *state, double *unused, double *out_1847379140425648298);
void car_H_27(double *state, double *unused, double *out_79638600820663839);
void car_h_29(double *state, double *unused, double *out_3665461276683812659);
void car_H_29(double *state, double *unused, double *out_2605356055294153256);
void car_h_28(double *state, double *unused, double *out_9164221498855202505);
void car_H_28(double *state, double *unused, double *out_2477042961775377318);
void car_h_31(double *state, double *unused, double *out_6556689476724902462);
void car_H_31(double *state, double *unused, double *out_4790919668634895255);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}