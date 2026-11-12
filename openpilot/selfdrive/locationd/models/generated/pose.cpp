#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8152519913131250608) {
   out_8152519913131250608[0] = delta_x[0] + nom_x[0];
   out_8152519913131250608[1] = delta_x[1] + nom_x[1];
   out_8152519913131250608[2] = delta_x[2] + nom_x[2];
   out_8152519913131250608[3] = delta_x[3] + nom_x[3];
   out_8152519913131250608[4] = delta_x[4] + nom_x[4];
   out_8152519913131250608[5] = delta_x[5] + nom_x[5];
   out_8152519913131250608[6] = delta_x[6] + nom_x[6];
   out_8152519913131250608[7] = delta_x[7] + nom_x[7];
   out_8152519913131250608[8] = delta_x[8] + nom_x[8];
   out_8152519913131250608[9] = delta_x[9] + nom_x[9];
   out_8152519913131250608[10] = delta_x[10] + nom_x[10];
   out_8152519913131250608[11] = delta_x[11] + nom_x[11];
   out_8152519913131250608[12] = delta_x[12] + nom_x[12];
   out_8152519913131250608[13] = delta_x[13] + nom_x[13];
   out_8152519913131250608[14] = delta_x[14] + nom_x[14];
   out_8152519913131250608[15] = delta_x[15] + nom_x[15];
   out_8152519913131250608[16] = delta_x[16] + nom_x[16];
   out_8152519913131250608[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1374580968925525788) {
   out_1374580968925525788[0] = -nom_x[0] + true_x[0];
   out_1374580968925525788[1] = -nom_x[1] + true_x[1];
   out_1374580968925525788[2] = -nom_x[2] + true_x[2];
   out_1374580968925525788[3] = -nom_x[3] + true_x[3];
   out_1374580968925525788[4] = -nom_x[4] + true_x[4];
   out_1374580968925525788[5] = -nom_x[5] + true_x[5];
   out_1374580968925525788[6] = -nom_x[6] + true_x[6];
   out_1374580968925525788[7] = -nom_x[7] + true_x[7];
   out_1374580968925525788[8] = -nom_x[8] + true_x[8];
   out_1374580968925525788[9] = -nom_x[9] + true_x[9];
   out_1374580968925525788[10] = -nom_x[10] + true_x[10];
   out_1374580968925525788[11] = -nom_x[11] + true_x[11];
   out_1374580968925525788[12] = -nom_x[12] + true_x[12];
   out_1374580968925525788[13] = -nom_x[13] + true_x[13];
   out_1374580968925525788[14] = -nom_x[14] + true_x[14];
   out_1374580968925525788[15] = -nom_x[15] + true_x[15];
   out_1374580968925525788[16] = -nom_x[16] + true_x[16];
   out_1374580968925525788[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3247388573912592584) {
   out_3247388573912592584[0] = 1.0;
   out_3247388573912592584[1] = 0.0;
   out_3247388573912592584[2] = 0.0;
   out_3247388573912592584[3] = 0.0;
   out_3247388573912592584[4] = 0.0;
   out_3247388573912592584[5] = 0.0;
   out_3247388573912592584[6] = 0.0;
   out_3247388573912592584[7] = 0.0;
   out_3247388573912592584[8] = 0.0;
   out_3247388573912592584[9] = 0.0;
   out_3247388573912592584[10] = 0.0;
   out_3247388573912592584[11] = 0.0;
   out_3247388573912592584[12] = 0.0;
   out_3247388573912592584[13] = 0.0;
   out_3247388573912592584[14] = 0.0;
   out_3247388573912592584[15] = 0.0;
   out_3247388573912592584[16] = 0.0;
   out_3247388573912592584[17] = 0.0;
   out_3247388573912592584[18] = 0.0;
   out_3247388573912592584[19] = 1.0;
   out_3247388573912592584[20] = 0.0;
   out_3247388573912592584[21] = 0.0;
   out_3247388573912592584[22] = 0.0;
   out_3247388573912592584[23] = 0.0;
   out_3247388573912592584[24] = 0.0;
   out_3247388573912592584[25] = 0.0;
   out_3247388573912592584[26] = 0.0;
   out_3247388573912592584[27] = 0.0;
   out_3247388573912592584[28] = 0.0;
   out_3247388573912592584[29] = 0.0;
   out_3247388573912592584[30] = 0.0;
   out_3247388573912592584[31] = 0.0;
   out_3247388573912592584[32] = 0.0;
   out_3247388573912592584[33] = 0.0;
   out_3247388573912592584[34] = 0.0;
   out_3247388573912592584[35] = 0.0;
   out_3247388573912592584[36] = 0.0;
   out_3247388573912592584[37] = 0.0;
   out_3247388573912592584[38] = 1.0;
   out_3247388573912592584[39] = 0.0;
   out_3247388573912592584[40] = 0.0;
   out_3247388573912592584[41] = 0.0;
   out_3247388573912592584[42] = 0.0;
   out_3247388573912592584[43] = 0.0;
   out_3247388573912592584[44] = 0.0;
   out_3247388573912592584[45] = 0.0;
   out_3247388573912592584[46] = 0.0;
   out_3247388573912592584[47] = 0.0;
   out_3247388573912592584[48] = 0.0;
   out_3247388573912592584[49] = 0.0;
   out_3247388573912592584[50] = 0.0;
   out_3247388573912592584[51] = 0.0;
   out_3247388573912592584[52] = 0.0;
   out_3247388573912592584[53] = 0.0;
   out_3247388573912592584[54] = 0.0;
   out_3247388573912592584[55] = 0.0;
   out_3247388573912592584[56] = 0.0;
   out_3247388573912592584[57] = 1.0;
   out_3247388573912592584[58] = 0.0;
   out_3247388573912592584[59] = 0.0;
   out_3247388573912592584[60] = 0.0;
   out_3247388573912592584[61] = 0.0;
   out_3247388573912592584[62] = 0.0;
   out_3247388573912592584[63] = 0.0;
   out_3247388573912592584[64] = 0.0;
   out_3247388573912592584[65] = 0.0;
   out_3247388573912592584[66] = 0.0;
   out_3247388573912592584[67] = 0.0;
   out_3247388573912592584[68] = 0.0;
   out_3247388573912592584[69] = 0.0;
   out_3247388573912592584[70] = 0.0;
   out_3247388573912592584[71] = 0.0;
   out_3247388573912592584[72] = 0.0;
   out_3247388573912592584[73] = 0.0;
   out_3247388573912592584[74] = 0.0;
   out_3247388573912592584[75] = 0.0;
   out_3247388573912592584[76] = 1.0;
   out_3247388573912592584[77] = 0.0;
   out_3247388573912592584[78] = 0.0;
   out_3247388573912592584[79] = 0.0;
   out_3247388573912592584[80] = 0.0;
   out_3247388573912592584[81] = 0.0;
   out_3247388573912592584[82] = 0.0;
   out_3247388573912592584[83] = 0.0;
   out_3247388573912592584[84] = 0.0;
   out_3247388573912592584[85] = 0.0;
   out_3247388573912592584[86] = 0.0;
   out_3247388573912592584[87] = 0.0;
   out_3247388573912592584[88] = 0.0;
   out_3247388573912592584[89] = 0.0;
   out_3247388573912592584[90] = 0.0;
   out_3247388573912592584[91] = 0.0;
   out_3247388573912592584[92] = 0.0;
   out_3247388573912592584[93] = 0.0;
   out_3247388573912592584[94] = 0.0;
   out_3247388573912592584[95] = 1.0;
   out_3247388573912592584[96] = 0.0;
   out_3247388573912592584[97] = 0.0;
   out_3247388573912592584[98] = 0.0;
   out_3247388573912592584[99] = 0.0;
   out_3247388573912592584[100] = 0.0;
   out_3247388573912592584[101] = 0.0;
   out_3247388573912592584[102] = 0.0;
   out_3247388573912592584[103] = 0.0;
   out_3247388573912592584[104] = 0.0;
   out_3247388573912592584[105] = 0.0;
   out_3247388573912592584[106] = 0.0;
   out_3247388573912592584[107] = 0.0;
   out_3247388573912592584[108] = 0.0;
   out_3247388573912592584[109] = 0.0;
   out_3247388573912592584[110] = 0.0;
   out_3247388573912592584[111] = 0.0;
   out_3247388573912592584[112] = 0.0;
   out_3247388573912592584[113] = 0.0;
   out_3247388573912592584[114] = 1.0;
   out_3247388573912592584[115] = 0.0;
   out_3247388573912592584[116] = 0.0;
   out_3247388573912592584[117] = 0.0;
   out_3247388573912592584[118] = 0.0;
   out_3247388573912592584[119] = 0.0;
   out_3247388573912592584[120] = 0.0;
   out_3247388573912592584[121] = 0.0;
   out_3247388573912592584[122] = 0.0;
   out_3247388573912592584[123] = 0.0;
   out_3247388573912592584[124] = 0.0;
   out_3247388573912592584[125] = 0.0;
   out_3247388573912592584[126] = 0.0;
   out_3247388573912592584[127] = 0.0;
   out_3247388573912592584[128] = 0.0;
   out_3247388573912592584[129] = 0.0;
   out_3247388573912592584[130] = 0.0;
   out_3247388573912592584[131] = 0.0;
   out_3247388573912592584[132] = 0.0;
   out_3247388573912592584[133] = 1.0;
   out_3247388573912592584[134] = 0.0;
   out_3247388573912592584[135] = 0.0;
   out_3247388573912592584[136] = 0.0;
   out_3247388573912592584[137] = 0.0;
   out_3247388573912592584[138] = 0.0;
   out_3247388573912592584[139] = 0.0;
   out_3247388573912592584[140] = 0.0;
   out_3247388573912592584[141] = 0.0;
   out_3247388573912592584[142] = 0.0;
   out_3247388573912592584[143] = 0.0;
   out_3247388573912592584[144] = 0.0;
   out_3247388573912592584[145] = 0.0;
   out_3247388573912592584[146] = 0.0;
   out_3247388573912592584[147] = 0.0;
   out_3247388573912592584[148] = 0.0;
   out_3247388573912592584[149] = 0.0;
   out_3247388573912592584[150] = 0.0;
   out_3247388573912592584[151] = 0.0;
   out_3247388573912592584[152] = 1.0;
   out_3247388573912592584[153] = 0.0;
   out_3247388573912592584[154] = 0.0;
   out_3247388573912592584[155] = 0.0;
   out_3247388573912592584[156] = 0.0;
   out_3247388573912592584[157] = 0.0;
   out_3247388573912592584[158] = 0.0;
   out_3247388573912592584[159] = 0.0;
   out_3247388573912592584[160] = 0.0;
   out_3247388573912592584[161] = 0.0;
   out_3247388573912592584[162] = 0.0;
   out_3247388573912592584[163] = 0.0;
   out_3247388573912592584[164] = 0.0;
   out_3247388573912592584[165] = 0.0;
   out_3247388573912592584[166] = 0.0;
   out_3247388573912592584[167] = 0.0;
   out_3247388573912592584[168] = 0.0;
   out_3247388573912592584[169] = 0.0;
   out_3247388573912592584[170] = 0.0;
   out_3247388573912592584[171] = 1.0;
   out_3247388573912592584[172] = 0.0;
   out_3247388573912592584[173] = 0.0;
   out_3247388573912592584[174] = 0.0;
   out_3247388573912592584[175] = 0.0;
   out_3247388573912592584[176] = 0.0;
   out_3247388573912592584[177] = 0.0;
   out_3247388573912592584[178] = 0.0;
   out_3247388573912592584[179] = 0.0;
   out_3247388573912592584[180] = 0.0;
   out_3247388573912592584[181] = 0.0;
   out_3247388573912592584[182] = 0.0;
   out_3247388573912592584[183] = 0.0;
   out_3247388573912592584[184] = 0.0;
   out_3247388573912592584[185] = 0.0;
   out_3247388573912592584[186] = 0.0;
   out_3247388573912592584[187] = 0.0;
   out_3247388573912592584[188] = 0.0;
   out_3247388573912592584[189] = 0.0;
   out_3247388573912592584[190] = 1.0;
   out_3247388573912592584[191] = 0.0;
   out_3247388573912592584[192] = 0.0;
   out_3247388573912592584[193] = 0.0;
   out_3247388573912592584[194] = 0.0;
   out_3247388573912592584[195] = 0.0;
   out_3247388573912592584[196] = 0.0;
   out_3247388573912592584[197] = 0.0;
   out_3247388573912592584[198] = 0.0;
   out_3247388573912592584[199] = 0.0;
   out_3247388573912592584[200] = 0.0;
   out_3247388573912592584[201] = 0.0;
   out_3247388573912592584[202] = 0.0;
   out_3247388573912592584[203] = 0.0;
   out_3247388573912592584[204] = 0.0;
   out_3247388573912592584[205] = 0.0;
   out_3247388573912592584[206] = 0.0;
   out_3247388573912592584[207] = 0.0;
   out_3247388573912592584[208] = 0.0;
   out_3247388573912592584[209] = 1.0;
   out_3247388573912592584[210] = 0.0;
   out_3247388573912592584[211] = 0.0;
   out_3247388573912592584[212] = 0.0;
   out_3247388573912592584[213] = 0.0;
   out_3247388573912592584[214] = 0.0;
   out_3247388573912592584[215] = 0.0;
   out_3247388573912592584[216] = 0.0;
   out_3247388573912592584[217] = 0.0;
   out_3247388573912592584[218] = 0.0;
   out_3247388573912592584[219] = 0.0;
   out_3247388573912592584[220] = 0.0;
   out_3247388573912592584[221] = 0.0;
   out_3247388573912592584[222] = 0.0;
   out_3247388573912592584[223] = 0.0;
   out_3247388573912592584[224] = 0.0;
   out_3247388573912592584[225] = 0.0;
   out_3247388573912592584[226] = 0.0;
   out_3247388573912592584[227] = 0.0;
   out_3247388573912592584[228] = 1.0;
   out_3247388573912592584[229] = 0.0;
   out_3247388573912592584[230] = 0.0;
   out_3247388573912592584[231] = 0.0;
   out_3247388573912592584[232] = 0.0;
   out_3247388573912592584[233] = 0.0;
   out_3247388573912592584[234] = 0.0;
   out_3247388573912592584[235] = 0.0;
   out_3247388573912592584[236] = 0.0;
   out_3247388573912592584[237] = 0.0;
   out_3247388573912592584[238] = 0.0;
   out_3247388573912592584[239] = 0.0;
   out_3247388573912592584[240] = 0.0;
   out_3247388573912592584[241] = 0.0;
   out_3247388573912592584[242] = 0.0;
   out_3247388573912592584[243] = 0.0;
   out_3247388573912592584[244] = 0.0;
   out_3247388573912592584[245] = 0.0;
   out_3247388573912592584[246] = 0.0;
   out_3247388573912592584[247] = 1.0;
   out_3247388573912592584[248] = 0.0;
   out_3247388573912592584[249] = 0.0;
   out_3247388573912592584[250] = 0.0;
   out_3247388573912592584[251] = 0.0;
   out_3247388573912592584[252] = 0.0;
   out_3247388573912592584[253] = 0.0;
   out_3247388573912592584[254] = 0.0;
   out_3247388573912592584[255] = 0.0;
   out_3247388573912592584[256] = 0.0;
   out_3247388573912592584[257] = 0.0;
   out_3247388573912592584[258] = 0.0;
   out_3247388573912592584[259] = 0.0;
   out_3247388573912592584[260] = 0.0;
   out_3247388573912592584[261] = 0.0;
   out_3247388573912592584[262] = 0.0;
   out_3247388573912592584[263] = 0.0;
   out_3247388573912592584[264] = 0.0;
   out_3247388573912592584[265] = 0.0;
   out_3247388573912592584[266] = 1.0;
   out_3247388573912592584[267] = 0.0;
   out_3247388573912592584[268] = 0.0;
   out_3247388573912592584[269] = 0.0;
   out_3247388573912592584[270] = 0.0;
   out_3247388573912592584[271] = 0.0;
   out_3247388573912592584[272] = 0.0;
   out_3247388573912592584[273] = 0.0;
   out_3247388573912592584[274] = 0.0;
   out_3247388573912592584[275] = 0.0;
   out_3247388573912592584[276] = 0.0;
   out_3247388573912592584[277] = 0.0;
   out_3247388573912592584[278] = 0.0;
   out_3247388573912592584[279] = 0.0;
   out_3247388573912592584[280] = 0.0;
   out_3247388573912592584[281] = 0.0;
   out_3247388573912592584[282] = 0.0;
   out_3247388573912592584[283] = 0.0;
   out_3247388573912592584[284] = 0.0;
   out_3247388573912592584[285] = 1.0;
   out_3247388573912592584[286] = 0.0;
   out_3247388573912592584[287] = 0.0;
   out_3247388573912592584[288] = 0.0;
   out_3247388573912592584[289] = 0.0;
   out_3247388573912592584[290] = 0.0;
   out_3247388573912592584[291] = 0.0;
   out_3247388573912592584[292] = 0.0;
   out_3247388573912592584[293] = 0.0;
   out_3247388573912592584[294] = 0.0;
   out_3247388573912592584[295] = 0.0;
   out_3247388573912592584[296] = 0.0;
   out_3247388573912592584[297] = 0.0;
   out_3247388573912592584[298] = 0.0;
   out_3247388573912592584[299] = 0.0;
   out_3247388573912592584[300] = 0.0;
   out_3247388573912592584[301] = 0.0;
   out_3247388573912592584[302] = 0.0;
   out_3247388573912592584[303] = 0.0;
   out_3247388573912592584[304] = 1.0;
   out_3247388573912592584[305] = 0.0;
   out_3247388573912592584[306] = 0.0;
   out_3247388573912592584[307] = 0.0;
   out_3247388573912592584[308] = 0.0;
   out_3247388573912592584[309] = 0.0;
   out_3247388573912592584[310] = 0.0;
   out_3247388573912592584[311] = 0.0;
   out_3247388573912592584[312] = 0.0;
   out_3247388573912592584[313] = 0.0;
   out_3247388573912592584[314] = 0.0;
   out_3247388573912592584[315] = 0.0;
   out_3247388573912592584[316] = 0.0;
   out_3247388573912592584[317] = 0.0;
   out_3247388573912592584[318] = 0.0;
   out_3247388573912592584[319] = 0.0;
   out_3247388573912592584[320] = 0.0;
   out_3247388573912592584[321] = 0.0;
   out_3247388573912592584[322] = 0.0;
   out_3247388573912592584[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1702150847149490218) {
   out_1702150847149490218[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1702150847149490218[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1702150847149490218[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1702150847149490218[3] = dt*state[12] + state[3];
   out_1702150847149490218[4] = dt*state[13] + state[4];
   out_1702150847149490218[5] = dt*state[14] + state[5];
   out_1702150847149490218[6] = state[6];
   out_1702150847149490218[7] = state[7];
   out_1702150847149490218[8] = state[8];
   out_1702150847149490218[9] = state[9];
   out_1702150847149490218[10] = state[10];
   out_1702150847149490218[11] = state[11];
   out_1702150847149490218[12] = state[12];
   out_1702150847149490218[13] = state[13];
   out_1702150847149490218[14] = state[14];
   out_1702150847149490218[15] = state[15];
   out_1702150847149490218[16] = state[16];
   out_1702150847149490218[17] = state[17];
}
void F_fun(double *state, double dt, double *out_310091906341172926) {
   out_310091906341172926[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_310091906341172926[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_310091906341172926[2] = 0;
   out_310091906341172926[3] = 0;
   out_310091906341172926[4] = 0;
   out_310091906341172926[5] = 0;
   out_310091906341172926[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_310091906341172926[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_310091906341172926[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_310091906341172926[9] = 0;
   out_310091906341172926[10] = 0;
   out_310091906341172926[11] = 0;
   out_310091906341172926[12] = 0;
   out_310091906341172926[13] = 0;
   out_310091906341172926[14] = 0;
   out_310091906341172926[15] = 0;
   out_310091906341172926[16] = 0;
   out_310091906341172926[17] = 0;
   out_310091906341172926[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_310091906341172926[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_310091906341172926[20] = 0;
   out_310091906341172926[21] = 0;
   out_310091906341172926[22] = 0;
   out_310091906341172926[23] = 0;
   out_310091906341172926[24] = 0;
   out_310091906341172926[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_310091906341172926[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_310091906341172926[27] = 0;
   out_310091906341172926[28] = 0;
   out_310091906341172926[29] = 0;
   out_310091906341172926[30] = 0;
   out_310091906341172926[31] = 0;
   out_310091906341172926[32] = 0;
   out_310091906341172926[33] = 0;
   out_310091906341172926[34] = 0;
   out_310091906341172926[35] = 0;
   out_310091906341172926[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_310091906341172926[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_310091906341172926[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_310091906341172926[39] = 0;
   out_310091906341172926[40] = 0;
   out_310091906341172926[41] = 0;
   out_310091906341172926[42] = 0;
   out_310091906341172926[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_310091906341172926[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_310091906341172926[45] = 0;
   out_310091906341172926[46] = 0;
   out_310091906341172926[47] = 0;
   out_310091906341172926[48] = 0;
   out_310091906341172926[49] = 0;
   out_310091906341172926[50] = 0;
   out_310091906341172926[51] = 0;
   out_310091906341172926[52] = 0;
   out_310091906341172926[53] = 0;
   out_310091906341172926[54] = 0;
   out_310091906341172926[55] = 0;
   out_310091906341172926[56] = 0;
   out_310091906341172926[57] = 1;
   out_310091906341172926[58] = 0;
   out_310091906341172926[59] = 0;
   out_310091906341172926[60] = 0;
   out_310091906341172926[61] = 0;
   out_310091906341172926[62] = 0;
   out_310091906341172926[63] = 0;
   out_310091906341172926[64] = 0;
   out_310091906341172926[65] = 0;
   out_310091906341172926[66] = dt;
   out_310091906341172926[67] = 0;
   out_310091906341172926[68] = 0;
   out_310091906341172926[69] = 0;
   out_310091906341172926[70] = 0;
   out_310091906341172926[71] = 0;
   out_310091906341172926[72] = 0;
   out_310091906341172926[73] = 0;
   out_310091906341172926[74] = 0;
   out_310091906341172926[75] = 0;
   out_310091906341172926[76] = 1;
   out_310091906341172926[77] = 0;
   out_310091906341172926[78] = 0;
   out_310091906341172926[79] = 0;
   out_310091906341172926[80] = 0;
   out_310091906341172926[81] = 0;
   out_310091906341172926[82] = 0;
   out_310091906341172926[83] = 0;
   out_310091906341172926[84] = 0;
   out_310091906341172926[85] = dt;
   out_310091906341172926[86] = 0;
   out_310091906341172926[87] = 0;
   out_310091906341172926[88] = 0;
   out_310091906341172926[89] = 0;
   out_310091906341172926[90] = 0;
   out_310091906341172926[91] = 0;
   out_310091906341172926[92] = 0;
   out_310091906341172926[93] = 0;
   out_310091906341172926[94] = 0;
   out_310091906341172926[95] = 1;
   out_310091906341172926[96] = 0;
   out_310091906341172926[97] = 0;
   out_310091906341172926[98] = 0;
   out_310091906341172926[99] = 0;
   out_310091906341172926[100] = 0;
   out_310091906341172926[101] = 0;
   out_310091906341172926[102] = 0;
   out_310091906341172926[103] = 0;
   out_310091906341172926[104] = dt;
   out_310091906341172926[105] = 0;
   out_310091906341172926[106] = 0;
   out_310091906341172926[107] = 0;
   out_310091906341172926[108] = 0;
   out_310091906341172926[109] = 0;
   out_310091906341172926[110] = 0;
   out_310091906341172926[111] = 0;
   out_310091906341172926[112] = 0;
   out_310091906341172926[113] = 0;
   out_310091906341172926[114] = 1;
   out_310091906341172926[115] = 0;
   out_310091906341172926[116] = 0;
   out_310091906341172926[117] = 0;
   out_310091906341172926[118] = 0;
   out_310091906341172926[119] = 0;
   out_310091906341172926[120] = 0;
   out_310091906341172926[121] = 0;
   out_310091906341172926[122] = 0;
   out_310091906341172926[123] = 0;
   out_310091906341172926[124] = 0;
   out_310091906341172926[125] = 0;
   out_310091906341172926[126] = 0;
   out_310091906341172926[127] = 0;
   out_310091906341172926[128] = 0;
   out_310091906341172926[129] = 0;
   out_310091906341172926[130] = 0;
   out_310091906341172926[131] = 0;
   out_310091906341172926[132] = 0;
   out_310091906341172926[133] = 1;
   out_310091906341172926[134] = 0;
   out_310091906341172926[135] = 0;
   out_310091906341172926[136] = 0;
   out_310091906341172926[137] = 0;
   out_310091906341172926[138] = 0;
   out_310091906341172926[139] = 0;
   out_310091906341172926[140] = 0;
   out_310091906341172926[141] = 0;
   out_310091906341172926[142] = 0;
   out_310091906341172926[143] = 0;
   out_310091906341172926[144] = 0;
   out_310091906341172926[145] = 0;
   out_310091906341172926[146] = 0;
   out_310091906341172926[147] = 0;
   out_310091906341172926[148] = 0;
   out_310091906341172926[149] = 0;
   out_310091906341172926[150] = 0;
   out_310091906341172926[151] = 0;
   out_310091906341172926[152] = 1;
   out_310091906341172926[153] = 0;
   out_310091906341172926[154] = 0;
   out_310091906341172926[155] = 0;
   out_310091906341172926[156] = 0;
   out_310091906341172926[157] = 0;
   out_310091906341172926[158] = 0;
   out_310091906341172926[159] = 0;
   out_310091906341172926[160] = 0;
   out_310091906341172926[161] = 0;
   out_310091906341172926[162] = 0;
   out_310091906341172926[163] = 0;
   out_310091906341172926[164] = 0;
   out_310091906341172926[165] = 0;
   out_310091906341172926[166] = 0;
   out_310091906341172926[167] = 0;
   out_310091906341172926[168] = 0;
   out_310091906341172926[169] = 0;
   out_310091906341172926[170] = 0;
   out_310091906341172926[171] = 1;
   out_310091906341172926[172] = 0;
   out_310091906341172926[173] = 0;
   out_310091906341172926[174] = 0;
   out_310091906341172926[175] = 0;
   out_310091906341172926[176] = 0;
   out_310091906341172926[177] = 0;
   out_310091906341172926[178] = 0;
   out_310091906341172926[179] = 0;
   out_310091906341172926[180] = 0;
   out_310091906341172926[181] = 0;
   out_310091906341172926[182] = 0;
   out_310091906341172926[183] = 0;
   out_310091906341172926[184] = 0;
   out_310091906341172926[185] = 0;
   out_310091906341172926[186] = 0;
   out_310091906341172926[187] = 0;
   out_310091906341172926[188] = 0;
   out_310091906341172926[189] = 0;
   out_310091906341172926[190] = 1;
   out_310091906341172926[191] = 0;
   out_310091906341172926[192] = 0;
   out_310091906341172926[193] = 0;
   out_310091906341172926[194] = 0;
   out_310091906341172926[195] = 0;
   out_310091906341172926[196] = 0;
   out_310091906341172926[197] = 0;
   out_310091906341172926[198] = 0;
   out_310091906341172926[199] = 0;
   out_310091906341172926[200] = 0;
   out_310091906341172926[201] = 0;
   out_310091906341172926[202] = 0;
   out_310091906341172926[203] = 0;
   out_310091906341172926[204] = 0;
   out_310091906341172926[205] = 0;
   out_310091906341172926[206] = 0;
   out_310091906341172926[207] = 0;
   out_310091906341172926[208] = 0;
   out_310091906341172926[209] = 1;
   out_310091906341172926[210] = 0;
   out_310091906341172926[211] = 0;
   out_310091906341172926[212] = 0;
   out_310091906341172926[213] = 0;
   out_310091906341172926[214] = 0;
   out_310091906341172926[215] = 0;
   out_310091906341172926[216] = 0;
   out_310091906341172926[217] = 0;
   out_310091906341172926[218] = 0;
   out_310091906341172926[219] = 0;
   out_310091906341172926[220] = 0;
   out_310091906341172926[221] = 0;
   out_310091906341172926[222] = 0;
   out_310091906341172926[223] = 0;
   out_310091906341172926[224] = 0;
   out_310091906341172926[225] = 0;
   out_310091906341172926[226] = 0;
   out_310091906341172926[227] = 0;
   out_310091906341172926[228] = 1;
   out_310091906341172926[229] = 0;
   out_310091906341172926[230] = 0;
   out_310091906341172926[231] = 0;
   out_310091906341172926[232] = 0;
   out_310091906341172926[233] = 0;
   out_310091906341172926[234] = 0;
   out_310091906341172926[235] = 0;
   out_310091906341172926[236] = 0;
   out_310091906341172926[237] = 0;
   out_310091906341172926[238] = 0;
   out_310091906341172926[239] = 0;
   out_310091906341172926[240] = 0;
   out_310091906341172926[241] = 0;
   out_310091906341172926[242] = 0;
   out_310091906341172926[243] = 0;
   out_310091906341172926[244] = 0;
   out_310091906341172926[245] = 0;
   out_310091906341172926[246] = 0;
   out_310091906341172926[247] = 1;
   out_310091906341172926[248] = 0;
   out_310091906341172926[249] = 0;
   out_310091906341172926[250] = 0;
   out_310091906341172926[251] = 0;
   out_310091906341172926[252] = 0;
   out_310091906341172926[253] = 0;
   out_310091906341172926[254] = 0;
   out_310091906341172926[255] = 0;
   out_310091906341172926[256] = 0;
   out_310091906341172926[257] = 0;
   out_310091906341172926[258] = 0;
   out_310091906341172926[259] = 0;
   out_310091906341172926[260] = 0;
   out_310091906341172926[261] = 0;
   out_310091906341172926[262] = 0;
   out_310091906341172926[263] = 0;
   out_310091906341172926[264] = 0;
   out_310091906341172926[265] = 0;
   out_310091906341172926[266] = 1;
   out_310091906341172926[267] = 0;
   out_310091906341172926[268] = 0;
   out_310091906341172926[269] = 0;
   out_310091906341172926[270] = 0;
   out_310091906341172926[271] = 0;
   out_310091906341172926[272] = 0;
   out_310091906341172926[273] = 0;
   out_310091906341172926[274] = 0;
   out_310091906341172926[275] = 0;
   out_310091906341172926[276] = 0;
   out_310091906341172926[277] = 0;
   out_310091906341172926[278] = 0;
   out_310091906341172926[279] = 0;
   out_310091906341172926[280] = 0;
   out_310091906341172926[281] = 0;
   out_310091906341172926[282] = 0;
   out_310091906341172926[283] = 0;
   out_310091906341172926[284] = 0;
   out_310091906341172926[285] = 1;
   out_310091906341172926[286] = 0;
   out_310091906341172926[287] = 0;
   out_310091906341172926[288] = 0;
   out_310091906341172926[289] = 0;
   out_310091906341172926[290] = 0;
   out_310091906341172926[291] = 0;
   out_310091906341172926[292] = 0;
   out_310091906341172926[293] = 0;
   out_310091906341172926[294] = 0;
   out_310091906341172926[295] = 0;
   out_310091906341172926[296] = 0;
   out_310091906341172926[297] = 0;
   out_310091906341172926[298] = 0;
   out_310091906341172926[299] = 0;
   out_310091906341172926[300] = 0;
   out_310091906341172926[301] = 0;
   out_310091906341172926[302] = 0;
   out_310091906341172926[303] = 0;
   out_310091906341172926[304] = 1;
   out_310091906341172926[305] = 0;
   out_310091906341172926[306] = 0;
   out_310091906341172926[307] = 0;
   out_310091906341172926[308] = 0;
   out_310091906341172926[309] = 0;
   out_310091906341172926[310] = 0;
   out_310091906341172926[311] = 0;
   out_310091906341172926[312] = 0;
   out_310091906341172926[313] = 0;
   out_310091906341172926[314] = 0;
   out_310091906341172926[315] = 0;
   out_310091906341172926[316] = 0;
   out_310091906341172926[317] = 0;
   out_310091906341172926[318] = 0;
   out_310091906341172926[319] = 0;
   out_310091906341172926[320] = 0;
   out_310091906341172926[321] = 0;
   out_310091906341172926[322] = 0;
   out_310091906341172926[323] = 1;
}
void h_4(double *state, double *unused, double *out_6653350053749401804) {
   out_6653350053749401804[0] = state[6] + state[9];
   out_6653350053749401804[1] = state[7] + state[10];
   out_6653350053749401804[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2191876023575036859) {
   out_2191876023575036859[0] = 0;
   out_2191876023575036859[1] = 0;
   out_2191876023575036859[2] = 0;
   out_2191876023575036859[3] = 0;
   out_2191876023575036859[4] = 0;
   out_2191876023575036859[5] = 0;
   out_2191876023575036859[6] = 1;
   out_2191876023575036859[7] = 0;
   out_2191876023575036859[8] = 0;
   out_2191876023575036859[9] = 1;
   out_2191876023575036859[10] = 0;
   out_2191876023575036859[11] = 0;
   out_2191876023575036859[12] = 0;
   out_2191876023575036859[13] = 0;
   out_2191876023575036859[14] = 0;
   out_2191876023575036859[15] = 0;
   out_2191876023575036859[16] = 0;
   out_2191876023575036859[17] = 0;
   out_2191876023575036859[18] = 0;
   out_2191876023575036859[19] = 0;
   out_2191876023575036859[20] = 0;
   out_2191876023575036859[21] = 0;
   out_2191876023575036859[22] = 0;
   out_2191876023575036859[23] = 0;
   out_2191876023575036859[24] = 0;
   out_2191876023575036859[25] = 1;
   out_2191876023575036859[26] = 0;
   out_2191876023575036859[27] = 0;
   out_2191876023575036859[28] = 1;
   out_2191876023575036859[29] = 0;
   out_2191876023575036859[30] = 0;
   out_2191876023575036859[31] = 0;
   out_2191876023575036859[32] = 0;
   out_2191876023575036859[33] = 0;
   out_2191876023575036859[34] = 0;
   out_2191876023575036859[35] = 0;
   out_2191876023575036859[36] = 0;
   out_2191876023575036859[37] = 0;
   out_2191876023575036859[38] = 0;
   out_2191876023575036859[39] = 0;
   out_2191876023575036859[40] = 0;
   out_2191876023575036859[41] = 0;
   out_2191876023575036859[42] = 0;
   out_2191876023575036859[43] = 0;
   out_2191876023575036859[44] = 1;
   out_2191876023575036859[45] = 0;
   out_2191876023575036859[46] = 0;
   out_2191876023575036859[47] = 1;
   out_2191876023575036859[48] = 0;
   out_2191876023575036859[49] = 0;
   out_2191876023575036859[50] = 0;
   out_2191876023575036859[51] = 0;
   out_2191876023575036859[52] = 0;
   out_2191876023575036859[53] = 0;
}
void h_10(double *state, double *unused, double *out_673524606329606226) {
   out_673524606329606226[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_673524606329606226[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_673524606329606226[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2175785760805776054) {
   out_2175785760805776054[0] = 0;
   out_2175785760805776054[1] = 9.8100000000000005*cos(state[1]);
   out_2175785760805776054[2] = 0;
   out_2175785760805776054[3] = 0;
   out_2175785760805776054[4] = -state[8];
   out_2175785760805776054[5] = state[7];
   out_2175785760805776054[6] = 0;
   out_2175785760805776054[7] = state[5];
   out_2175785760805776054[8] = -state[4];
   out_2175785760805776054[9] = 0;
   out_2175785760805776054[10] = 0;
   out_2175785760805776054[11] = 0;
   out_2175785760805776054[12] = 1;
   out_2175785760805776054[13] = 0;
   out_2175785760805776054[14] = 0;
   out_2175785760805776054[15] = 1;
   out_2175785760805776054[16] = 0;
   out_2175785760805776054[17] = 0;
   out_2175785760805776054[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2175785760805776054[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2175785760805776054[20] = 0;
   out_2175785760805776054[21] = state[8];
   out_2175785760805776054[22] = 0;
   out_2175785760805776054[23] = -state[6];
   out_2175785760805776054[24] = -state[5];
   out_2175785760805776054[25] = 0;
   out_2175785760805776054[26] = state[3];
   out_2175785760805776054[27] = 0;
   out_2175785760805776054[28] = 0;
   out_2175785760805776054[29] = 0;
   out_2175785760805776054[30] = 0;
   out_2175785760805776054[31] = 1;
   out_2175785760805776054[32] = 0;
   out_2175785760805776054[33] = 0;
   out_2175785760805776054[34] = 1;
   out_2175785760805776054[35] = 0;
   out_2175785760805776054[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2175785760805776054[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2175785760805776054[38] = 0;
   out_2175785760805776054[39] = -state[7];
   out_2175785760805776054[40] = state[6];
   out_2175785760805776054[41] = 0;
   out_2175785760805776054[42] = state[4];
   out_2175785760805776054[43] = -state[3];
   out_2175785760805776054[44] = 0;
   out_2175785760805776054[45] = 0;
   out_2175785760805776054[46] = 0;
   out_2175785760805776054[47] = 0;
   out_2175785760805776054[48] = 0;
   out_2175785760805776054[49] = 0;
   out_2175785760805776054[50] = 1;
   out_2175785760805776054[51] = 0;
   out_2175785760805776054[52] = 0;
   out_2175785760805776054[53] = 1;
}
void h_13(double *state, double *unused, double *out_294251852541502500) {
   out_294251852541502500[0] = state[3];
   out_294251852541502500[1] = state[4];
   out_294251852541502500[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5404149848907369660) {
   out_5404149848907369660[0] = 0;
   out_5404149848907369660[1] = 0;
   out_5404149848907369660[2] = 0;
   out_5404149848907369660[3] = 1;
   out_5404149848907369660[4] = 0;
   out_5404149848907369660[5] = 0;
   out_5404149848907369660[6] = 0;
   out_5404149848907369660[7] = 0;
   out_5404149848907369660[8] = 0;
   out_5404149848907369660[9] = 0;
   out_5404149848907369660[10] = 0;
   out_5404149848907369660[11] = 0;
   out_5404149848907369660[12] = 0;
   out_5404149848907369660[13] = 0;
   out_5404149848907369660[14] = 0;
   out_5404149848907369660[15] = 0;
   out_5404149848907369660[16] = 0;
   out_5404149848907369660[17] = 0;
   out_5404149848907369660[18] = 0;
   out_5404149848907369660[19] = 0;
   out_5404149848907369660[20] = 0;
   out_5404149848907369660[21] = 0;
   out_5404149848907369660[22] = 1;
   out_5404149848907369660[23] = 0;
   out_5404149848907369660[24] = 0;
   out_5404149848907369660[25] = 0;
   out_5404149848907369660[26] = 0;
   out_5404149848907369660[27] = 0;
   out_5404149848907369660[28] = 0;
   out_5404149848907369660[29] = 0;
   out_5404149848907369660[30] = 0;
   out_5404149848907369660[31] = 0;
   out_5404149848907369660[32] = 0;
   out_5404149848907369660[33] = 0;
   out_5404149848907369660[34] = 0;
   out_5404149848907369660[35] = 0;
   out_5404149848907369660[36] = 0;
   out_5404149848907369660[37] = 0;
   out_5404149848907369660[38] = 0;
   out_5404149848907369660[39] = 0;
   out_5404149848907369660[40] = 0;
   out_5404149848907369660[41] = 1;
   out_5404149848907369660[42] = 0;
   out_5404149848907369660[43] = 0;
   out_5404149848907369660[44] = 0;
   out_5404149848907369660[45] = 0;
   out_5404149848907369660[46] = 0;
   out_5404149848907369660[47] = 0;
   out_5404149848907369660[48] = 0;
   out_5404149848907369660[49] = 0;
   out_5404149848907369660[50] = 0;
   out_5404149848907369660[51] = 0;
   out_5404149848907369660[52] = 0;
   out_5404149848907369660[53] = 0;
}
void h_14(double *state, double *unused, double *out_1394414573507401091) {
   out_1394414573507401091[0] = state[6];
   out_1394414573507401091[1] = state[7];
   out_1394414573507401091[2] = state[8];
}
void H_14(double *state, double *unused, double *out_890912408720335437) {
   out_890912408720335437[0] = 0;
   out_890912408720335437[1] = 0;
   out_890912408720335437[2] = 0;
   out_890912408720335437[3] = 0;
   out_890912408720335437[4] = 0;
   out_890912408720335437[5] = 0;
   out_890912408720335437[6] = 1;
   out_890912408720335437[7] = 0;
   out_890912408720335437[8] = 0;
   out_890912408720335437[9] = 0;
   out_890912408720335437[10] = 0;
   out_890912408720335437[11] = 0;
   out_890912408720335437[12] = 0;
   out_890912408720335437[13] = 0;
   out_890912408720335437[14] = 0;
   out_890912408720335437[15] = 0;
   out_890912408720335437[16] = 0;
   out_890912408720335437[17] = 0;
   out_890912408720335437[18] = 0;
   out_890912408720335437[19] = 0;
   out_890912408720335437[20] = 0;
   out_890912408720335437[21] = 0;
   out_890912408720335437[22] = 0;
   out_890912408720335437[23] = 0;
   out_890912408720335437[24] = 0;
   out_890912408720335437[25] = 1;
   out_890912408720335437[26] = 0;
   out_890912408720335437[27] = 0;
   out_890912408720335437[28] = 0;
   out_890912408720335437[29] = 0;
   out_890912408720335437[30] = 0;
   out_890912408720335437[31] = 0;
   out_890912408720335437[32] = 0;
   out_890912408720335437[33] = 0;
   out_890912408720335437[34] = 0;
   out_890912408720335437[35] = 0;
   out_890912408720335437[36] = 0;
   out_890912408720335437[37] = 0;
   out_890912408720335437[38] = 0;
   out_890912408720335437[39] = 0;
   out_890912408720335437[40] = 0;
   out_890912408720335437[41] = 0;
   out_890912408720335437[42] = 0;
   out_890912408720335437[43] = 0;
   out_890912408720335437[44] = 1;
   out_890912408720335437[45] = 0;
   out_890912408720335437[46] = 0;
   out_890912408720335437[47] = 0;
   out_890912408720335437[48] = 0;
   out_890912408720335437[49] = 0;
   out_890912408720335437[50] = 0;
   out_890912408720335437[51] = 0;
   out_890912408720335437[52] = 0;
   out_890912408720335437[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_8152519913131250608) {
  err_fun(nom_x, delta_x, out_8152519913131250608);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1374580968925525788) {
  inv_err_fun(nom_x, true_x, out_1374580968925525788);
}
void pose_H_mod_fun(double *state, double *out_3247388573912592584) {
  H_mod_fun(state, out_3247388573912592584);
}
void pose_f_fun(double *state, double dt, double *out_1702150847149490218) {
  f_fun(state,  dt, out_1702150847149490218);
}
void pose_F_fun(double *state, double dt, double *out_310091906341172926) {
  F_fun(state,  dt, out_310091906341172926);
}
void pose_h_4(double *state, double *unused, double *out_6653350053749401804) {
  h_4(state, unused, out_6653350053749401804);
}
void pose_H_4(double *state, double *unused, double *out_2191876023575036859) {
  H_4(state, unused, out_2191876023575036859);
}
void pose_h_10(double *state, double *unused, double *out_673524606329606226) {
  h_10(state, unused, out_673524606329606226);
}
void pose_H_10(double *state, double *unused, double *out_2175785760805776054) {
  H_10(state, unused, out_2175785760805776054);
}
void pose_h_13(double *state, double *unused, double *out_294251852541502500) {
  h_13(state, unused, out_294251852541502500);
}
void pose_H_13(double *state, double *unused, double *out_5404149848907369660) {
  H_13(state, unused, out_5404149848907369660);
}
void pose_h_14(double *state, double *unused, double *out_1394414573507401091) {
  h_14(state, unused, out_1394414573507401091);
}
void pose_H_14(double *state, double *unused, double *out_890912408720335437) {
  H_14(state, unused, out_890912408720335437);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
