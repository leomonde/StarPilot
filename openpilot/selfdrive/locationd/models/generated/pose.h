#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8152519913131250608);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1374580968925525788);
void pose_H_mod_fun(double *state, double *out_3247388573912592584);
void pose_f_fun(double *state, double dt, double *out_1702150847149490218);
void pose_F_fun(double *state, double dt, double *out_310091906341172926);
void pose_h_4(double *state, double *unused, double *out_6653350053749401804);
void pose_H_4(double *state, double *unused, double *out_2191876023575036859);
void pose_h_10(double *state, double *unused, double *out_673524606329606226);
void pose_H_10(double *state, double *unused, double *out_2175785760805776054);
void pose_h_13(double *state, double *unused, double *out_294251852541502500);
void pose_H_13(double *state, double *unused, double *out_5404149848907369660);
void pose_h_14(double *state, double *unused, double *out_1394414573507401091);
void pose_H_14(double *state, double *unused, double *out_890912408720335437);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}