/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 129 */
real_t state[ 129 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 9];
state[1] = acadoVariables.x[lRun1 * 9 + 1];
state[2] = acadoVariables.x[lRun1 * 9 + 2];
state[3] = acadoVariables.x[lRun1 * 9 + 3];
state[4] = acadoVariables.x[lRun1 * 9 + 4];
state[5] = acadoVariables.x[lRun1 * 9 + 5];
state[6] = acadoVariables.x[lRun1 * 9 + 6];
state[7] = acadoVariables.x[lRun1 * 9 + 7];
state[8] = acadoVariables.x[lRun1 * 9 + 8];

state[117] = acadoVariables.u[lRun1 * 3];
state[118] = acadoVariables.u[lRun1 * 3 + 1];
state[119] = acadoVariables.u[lRun1 * 3 + 2];
state[120] = acadoVariables.od[lRun1 * 9];
state[121] = acadoVariables.od[lRun1 * 9 + 1];
state[122] = acadoVariables.od[lRun1 * 9 + 2];
state[123] = acadoVariables.od[lRun1 * 9 + 3];
state[124] = acadoVariables.od[lRun1 * 9 + 4];
state[125] = acadoVariables.od[lRun1 * 9 + 5];
state[126] = acadoVariables.od[lRun1 * 9 + 6];
state[127] = acadoVariables.od[lRun1 * 9 + 7];
state[128] = acadoVariables.od[lRun1 * 9 + 8];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 9] = state[0] - acadoVariables.x[lRun1 * 9 + 9];
acadoWorkspace.d[lRun1 * 9 + 1] = state[1] - acadoVariables.x[lRun1 * 9 + 10];
acadoWorkspace.d[lRun1 * 9 + 2] = state[2] - acadoVariables.x[lRun1 * 9 + 11];
acadoWorkspace.d[lRun1 * 9 + 3] = state[3] - acadoVariables.x[lRun1 * 9 + 12];
acadoWorkspace.d[lRun1 * 9 + 4] = state[4] - acadoVariables.x[lRun1 * 9 + 13];
acadoWorkspace.d[lRun1 * 9 + 5] = state[5] - acadoVariables.x[lRun1 * 9 + 14];
acadoWorkspace.d[lRun1 * 9 + 6] = state[6] - acadoVariables.x[lRun1 * 9 + 15];
acadoWorkspace.d[lRun1 * 9 + 7] = state[7] - acadoVariables.x[lRun1 * 9 + 16];
acadoWorkspace.d[lRun1 * 9 + 8] = state[8] - acadoVariables.x[lRun1 * 9 + 17];

acadoWorkspace.evGx[lRun1 * 81] = state[9];
acadoWorkspace.evGx[lRun1 * 81 + 1] = state[10];
acadoWorkspace.evGx[lRun1 * 81 + 2] = state[11];
acadoWorkspace.evGx[lRun1 * 81 + 3] = state[12];
acadoWorkspace.evGx[lRun1 * 81 + 4] = state[13];
acadoWorkspace.evGx[lRun1 * 81 + 5] = state[14];
acadoWorkspace.evGx[lRun1 * 81 + 6] = state[15];
acadoWorkspace.evGx[lRun1 * 81 + 7] = state[16];
acadoWorkspace.evGx[lRun1 * 81 + 8] = state[17];
acadoWorkspace.evGx[lRun1 * 81 + 9] = state[18];
acadoWorkspace.evGx[lRun1 * 81 + 10] = state[19];
acadoWorkspace.evGx[lRun1 * 81 + 11] = state[20];
acadoWorkspace.evGx[lRun1 * 81 + 12] = state[21];
acadoWorkspace.evGx[lRun1 * 81 + 13] = state[22];
acadoWorkspace.evGx[lRun1 * 81 + 14] = state[23];
acadoWorkspace.evGx[lRun1 * 81 + 15] = state[24];
acadoWorkspace.evGx[lRun1 * 81 + 16] = state[25];
acadoWorkspace.evGx[lRun1 * 81 + 17] = state[26];
acadoWorkspace.evGx[lRun1 * 81 + 18] = state[27];
acadoWorkspace.evGx[lRun1 * 81 + 19] = state[28];
acadoWorkspace.evGx[lRun1 * 81 + 20] = state[29];
acadoWorkspace.evGx[lRun1 * 81 + 21] = state[30];
acadoWorkspace.evGx[lRun1 * 81 + 22] = state[31];
acadoWorkspace.evGx[lRun1 * 81 + 23] = state[32];
acadoWorkspace.evGx[lRun1 * 81 + 24] = state[33];
acadoWorkspace.evGx[lRun1 * 81 + 25] = state[34];
acadoWorkspace.evGx[lRun1 * 81 + 26] = state[35];
acadoWorkspace.evGx[lRun1 * 81 + 27] = state[36];
acadoWorkspace.evGx[lRun1 * 81 + 28] = state[37];
acadoWorkspace.evGx[lRun1 * 81 + 29] = state[38];
acadoWorkspace.evGx[lRun1 * 81 + 30] = state[39];
acadoWorkspace.evGx[lRun1 * 81 + 31] = state[40];
acadoWorkspace.evGx[lRun1 * 81 + 32] = state[41];
acadoWorkspace.evGx[lRun1 * 81 + 33] = state[42];
acadoWorkspace.evGx[lRun1 * 81 + 34] = state[43];
acadoWorkspace.evGx[lRun1 * 81 + 35] = state[44];
acadoWorkspace.evGx[lRun1 * 81 + 36] = state[45];
acadoWorkspace.evGx[lRun1 * 81 + 37] = state[46];
acadoWorkspace.evGx[lRun1 * 81 + 38] = state[47];
acadoWorkspace.evGx[lRun1 * 81 + 39] = state[48];
acadoWorkspace.evGx[lRun1 * 81 + 40] = state[49];
acadoWorkspace.evGx[lRun1 * 81 + 41] = state[50];
acadoWorkspace.evGx[lRun1 * 81 + 42] = state[51];
acadoWorkspace.evGx[lRun1 * 81 + 43] = state[52];
acadoWorkspace.evGx[lRun1 * 81 + 44] = state[53];
acadoWorkspace.evGx[lRun1 * 81 + 45] = state[54];
acadoWorkspace.evGx[lRun1 * 81 + 46] = state[55];
acadoWorkspace.evGx[lRun1 * 81 + 47] = state[56];
acadoWorkspace.evGx[lRun1 * 81 + 48] = state[57];
acadoWorkspace.evGx[lRun1 * 81 + 49] = state[58];
acadoWorkspace.evGx[lRun1 * 81 + 50] = state[59];
acadoWorkspace.evGx[lRun1 * 81 + 51] = state[60];
acadoWorkspace.evGx[lRun1 * 81 + 52] = state[61];
acadoWorkspace.evGx[lRun1 * 81 + 53] = state[62];
acadoWorkspace.evGx[lRun1 * 81 + 54] = state[63];
acadoWorkspace.evGx[lRun1 * 81 + 55] = state[64];
acadoWorkspace.evGx[lRun1 * 81 + 56] = state[65];
acadoWorkspace.evGx[lRun1 * 81 + 57] = state[66];
acadoWorkspace.evGx[lRun1 * 81 + 58] = state[67];
acadoWorkspace.evGx[lRun1 * 81 + 59] = state[68];
acadoWorkspace.evGx[lRun1 * 81 + 60] = state[69];
acadoWorkspace.evGx[lRun1 * 81 + 61] = state[70];
acadoWorkspace.evGx[lRun1 * 81 + 62] = state[71];
acadoWorkspace.evGx[lRun1 * 81 + 63] = state[72];
acadoWorkspace.evGx[lRun1 * 81 + 64] = state[73];
acadoWorkspace.evGx[lRun1 * 81 + 65] = state[74];
acadoWorkspace.evGx[lRun1 * 81 + 66] = state[75];
acadoWorkspace.evGx[lRun1 * 81 + 67] = state[76];
acadoWorkspace.evGx[lRun1 * 81 + 68] = state[77];
acadoWorkspace.evGx[lRun1 * 81 + 69] = state[78];
acadoWorkspace.evGx[lRun1 * 81 + 70] = state[79];
acadoWorkspace.evGx[lRun1 * 81 + 71] = state[80];
acadoWorkspace.evGx[lRun1 * 81 + 72] = state[81];
acadoWorkspace.evGx[lRun1 * 81 + 73] = state[82];
acadoWorkspace.evGx[lRun1 * 81 + 74] = state[83];
acadoWorkspace.evGx[lRun1 * 81 + 75] = state[84];
acadoWorkspace.evGx[lRun1 * 81 + 76] = state[85];
acadoWorkspace.evGx[lRun1 * 81 + 77] = state[86];
acadoWorkspace.evGx[lRun1 * 81 + 78] = state[87];
acadoWorkspace.evGx[lRun1 * 81 + 79] = state[88];
acadoWorkspace.evGx[lRun1 * 81 + 80] = state[89];

acadoWorkspace.evGu[lRun1 * 27] = state[90];
acadoWorkspace.evGu[lRun1 * 27 + 1] = state[91];
acadoWorkspace.evGu[lRun1 * 27 + 2] = state[92];
acadoWorkspace.evGu[lRun1 * 27 + 3] = state[93];
acadoWorkspace.evGu[lRun1 * 27 + 4] = state[94];
acadoWorkspace.evGu[lRun1 * 27 + 5] = state[95];
acadoWorkspace.evGu[lRun1 * 27 + 6] = state[96];
acadoWorkspace.evGu[lRun1 * 27 + 7] = state[97];
acadoWorkspace.evGu[lRun1 * 27 + 8] = state[98];
acadoWorkspace.evGu[lRun1 * 27 + 9] = state[99];
acadoWorkspace.evGu[lRun1 * 27 + 10] = state[100];
acadoWorkspace.evGu[lRun1 * 27 + 11] = state[101];
acadoWorkspace.evGu[lRun1 * 27 + 12] = state[102];
acadoWorkspace.evGu[lRun1 * 27 + 13] = state[103];
acadoWorkspace.evGu[lRun1 * 27 + 14] = state[104];
acadoWorkspace.evGu[lRun1 * 27 + 15] = state[105];
acadoWorkspace.evGu[lRun1 * 27 + 16] = state[106];
acadoWorkspace.evGu[lRun1 * 27 + 17] = state[107];
acadoWorkspace.evGu[lRun1 * 27 + 18] = state[108];
acadoWorkspace.evGu[lRun1 * 27 + 19] = state[109];
acadoWorkspace.evGu[lRun1 * 27 + 20] = state[110];
acadoWorkspace.evGu[lRun1 * 27 + 21] = state[111];
acadoWorkspace.evGu[lRun1 * 27 + 22] = state[112];
acadoWorkspace.evGu[lRun1 * 27 + 23] = state[113];
acadoWorkspace.evGu[lRun1 * 27 + 24] = state[114];
acadoWorkspace.evGu[lRun1 * 27 + 25] = state[115];
acadoWorkspace.evGu[lRun1 * 27 + 26] = state[116];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
out[6] = xd[3];
out[7] = xd[4];
out[8] = u[0];
out[9] = u[1];
out[10] = (u[2]-(real_t)(9.8065999999999995e+00));
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[6];
out[1] = xd[7];
out[2] = xd[8];
out[3] = xd[0];
out[4] = xd[1];
out[5] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[33];
tmpQ2[1] = +tmpObjS[34];
tmpQ2[2] = +tmpObjS[35];
tmpQ2[3] = +tmpObjS[36];
tmpQ2[4] = +tmpObjS[37];
tmpQ2[5] = +tmpObjS[38];
tmpQ2[6] = +tmpObjS[39];
tmpQ2[7] = +tmpObjS[40];
tmpQ2[8] = +tmpObjS[41];
tmpQ2[9] = +tmpObjS[42];
tmpQ2[10] = +tmpObjS[43];
tmpQ2[11] = +tmpObjS[44];
tmpQ2[12] = +tmpObjS[45];
tmpQ2[13] = +tmpObjS[46];
tmpQ2[14] = +tmpObjS[47];
tmpQ2[15] = +tmpObjS[48];
tmpQ2[16] = +tmpObjS[49];
tmpQ2[17] = +tmpObjS[50];
tmpQ2[18] = +tmpObjS[51];
tmpQ2[19] = +tmpObjS[52];
tmpQ2[20] = +tmpObjS[53];
tmpQ2[21] = +tmpObjS[54];
tmpQ2[22] = +tmpObjS[55];
tmpQ2[23] = +tmpObjS[56];
tmpQ2[24] = +tmpObjS[57];
tmpQ2[25] = +tmpObjS[58];
tmpQ2[26] = +tmpObjS[59];
tmpQ2[27] = +tmpObjS[60];
tmpQ2[28] = +tmpObjS[61];
tmpQ2[29] = +tmpObjS[62];
tmpQ2[30] = +tmpObjS[63];
tmpQ2[31] = +tmpObjS[64];
tmpQ2[32] = +tmpObjS[65];
tmpQ2[33] = +tmpObjS[66];
tmpQ2[34] = +tmpObjS[67];
tmpQ2[35] = +tmpObjS[68];
tmpQ2[36] = +tmpObjS[69];
tmpQ2[37] = +tmpObjS[70];
tmpQ2[38] = +tmpObjS[71];
tmpQ2[39] = +tmpObjS[72];
tmpQ2[40] = +tmpObjS[73];
tmpQ2[41] = +tmpObjS[74];
tmpQ2[42] = +tmpObjS[75];
tmpQ2[43] = +tmpObjS[76];
tmpQ2[44] = +tmpObjS[77];
tmpQ2[45] = +tmpObjS[78];
tmpQ2[46] = +tmpObjS[79];
tmpQ2[47] = +tmpObjS[80];
tmpQ2[48] = +tmpObjS[81];
tmpQ2[49] = +tmpObjS[82];
tmpQ2[50] = +tmpObjS[83];
tmpQ2[51] = +tmpObjS[84];
tmpQ2[52] = +tmpObjS[85];
tmpQ2[53] = +tmpObjS[86];
tmpQ2[54] = +tmpObjS[87];
tmpQ2[55] = 0.0;
;
tmpQ2[56] = 0.0;
;
tmpQ2[57] = 0.0;
;
tmpQ2[58] = 0.0;
;
tmpQ2[59] = 0.0;
;
tmpQ2[60] = 0.0;
;
tmpQ2[61] = 0.0;
;
tmpQ2[62] = 0.0;
;
tmpQ2[63] = 0.0;
;
tmpQ2[64] = 0.0;
;
tmpQ2[65] = 0.0;
;
tmpQ2[66] = +tmpObjS[0];
tmpQ2[67] = +tmpObjS[1];
tmpQ2[68] = +tmpObjS[2];
tmpQ2[69] = +tmpObjS[3];
tmpQ2[70] = +tmpObjS[4];
tmpQ2[71] = +tmpObjS[5];
tmpQ2[72] = +tmpObjS[6];
tmpQ2[73] = +tmpObjS[7];
tmpQ2[74] = +tmpObjS[8];
tmpQ2[75] = +tmpObjS[9];
tmpQ2[76] = +tmpObjS[10];
tmpQ2[77] = +tmpObjS[11];
tmpQ2[78] = +tmpObjS[12];
tmpQ2[79] = +tmpObjS[13];
tmpQ2[80] = +tmpObjS[14];
tmpQ2[81] = +tmpObjS[15];
tmpQ2[82] = +tmpObjS[16];
tmpQ2[83] = +tmpObjS[17];
tmpQ2[84] = +tmpObjS[18];
tmpQ2[85] = +tmpObjS[19];
tmpQ2[86] = +tmpObjS[20];
tmpQ2[87] = +tmpObjS[21];
tmpQ2[88] = +tmpObjS[22];
tmpQ2[89] = +tmpObjS[23];
tmpQ2[90] = +tmpObjS[24];
tmpQ2[91] = +tmpObjS[25];
tmpQ2[92] = +tmpObjS[26];
tmpQ2[93] = +tmpObjS[27];
tmpQ2[94] = +tmpObjS[28];
tmpQ2[95] = +tmpObjS[29];
tmpQ2[96] = +tmpObjS[30];
tmpQ2[97] = +tmpObjS[31];
tmpQ2[98] = +tmpObjS[32];
tmpQ1[0] = + tmpQ2[3];
tmpQ1[1] = + tmpQ2[4];
tmpQ1[2] = + tmpQ2[5];
tmpQ1[3] = + tmpQ2[6];
tmpQ1[4] = + tmpQ2[7];
tmpQ1[5] = 0.0;
;
tmpQ1[6] = + tmpQ2[0];
tmpQ1[7] = + tmpQ2[1];
tmpQ1[8] = + tmpQ2[2];
tmpQ1[9] = + tmpQ2[14];
tmpQ1[10] = + tmpQ2[15];
tmpQ1[11] = + tmpQ2[16];
tmpQ1[12] = + tmpQ2[17];
tmpQ1[13] = + tmpQ2[18];
tmpQ1[14] = 0.0;
;
tmpQ1[15] = + tmpQ2[11];
tmpQ1[16] = + tmpQ2[12];
tmpQ1[17] = + tmpQ2[13];
tmpQ1[18] = + tmpQ2[25];
tmpQ1[19] = + tmpQ2[26];
tmpQ1[20] = + tmpQ2[27];
tmpQ1[21] = + tmpQ2[28];
tmpQ1[22] = + tmpQ2[29];
tmpQ1[23] = 0.0;
;
tmpQ1[24] = + tmpQ2[22];
tmpQ1[25] = + tmpQ2[23];
tmpQ1[26] = + tmpQ2[24];
tmpQ1[27] = + tmpQ2[36];
tmpQ1[28] = + tmpQ2[37];
tmpQ1[29] = + tmpQ2[38];
tmpQ1[30] = + tmpQ2[39];
tmpQ1[31] = + tmpQ2[40];
tmpQ1[32] = 0.0;
;
tmpQ1[33] = + tmpQ2[33];
tmpQ1[34] = + tmpQ2[34];
tmpQ1[35] = + tmpQ2[35];
tmpQ1[36] = + tmpQ2[47];
tmpQ1[37] = + tmpQ2[48];
tmpQ1[38] = + tmpQ2[49];
tmpQ1[39] = + tmpQ2[50];
tmpQ1[40] = + tmpQ2[51];
tmpQ1[41] = 0.0;
;
tmpQ1[42] = + tmpQ2[44];
tmpQ1[43] = + tmpQ2[45];
tmpQ1[44] = + tmpQ2[46];
tmpQ1[45] = + tmpQ2[58];
tmpQ1[46] = + tmpQ2[59];
tmpQ1[47] = + tmpQ2[60];
tmpQ1[48] = + tmpQ2[61];
tmpQ1[49] = + tmpQ2[62];
tmpQ1[50] = 0.0;
;
tmpQ1[51] = + tmpQ2[55];
tmpQ1[52] = + tmpQ2[56];
tmpQ1[53] = + tmpQ2[57];
tmpQ1[54] = + tmpQ2[69];
tmpQ1[55] = + tmpQ2[70];
tmpQ1[56] = + tmpQ2[71];
tmpQ1[57] = + tmpQ2[72];
tmpQ1[58] = + tmpQ2[73];
tmpQ1[59] = 0.0;
;
tmpQ1[60] = + tmpQ2[66];
tmpQ1[61] = + tmpQ2[67];
tmpQ1[62] = + tmpQ2[68];
tmpQ1[63] = + tmpQ2[80];
tmpQ1[64] = + tmpQ2[81];
tmpQ1[65] = + tmpQ2[82];
tmpQ1[66] = + tmpQ2[83];
tmpQ1[67] = + tmpQ2[84];
tmpQ1[68] = 0.0;
;
tmpQ1[69] = + tmpQ2[77];
tmpQ1[70] = + tmpQ2[78];
tmpQ1[71] = + tmpQ2[79];
tmpQ1[72] = + tmpQ2[91];
tmpQ1[73] = + tmpQ2[92];
tmpQ1[74] = + tmpQ2[93];
tmpQ1[75] = + tmpQ2[94];
tmpQ1[76] = + tmpQ2[95];
tmpQ1[77] = 0.0;
;
tmpQ1[78] = + tmpQ2[88];
tmpQ1[79] = + tmpQ2[89];
tmpQ1[80] = + tmpQ2[90];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[88];
tmpR2[1] = +tmpObjS[89];
tmpR2[2] = +tmpObjS[90];
tmpR2[3] = +tmpObjS[91];
tmpR2[4] = +tmpObjS[92];
tmpR2[5] = +tmpObjS[93];
tmpR2[6] = +tmpObjS[94];
tmpR2[7] = +tmpObjS[95];
tmpR2[8] = +tmpObjS[96];
tmpR2[9] = +tmpObjS[97];
tmpR2[10] = +tmpObjS[98];
tmpR2[11] = +tmpObjS[99];
tmpR2[12] = +tmpObjS[100];
tmpR2[13] = +tmpObjS[101];
tmpR2[14] = +tmpObjS[102];
tmpR2[15] = +tmpObjS[103];
tmpR2[16] = +tmpObjS[104];
tmpR2[17] = +tmpObjS[105];
tmpR2[18] = +tmpObjS[106];
tmpR2[19] = +tmpObjS[107];
tmpR2[20] = +tmpObjS[108];
tmpR2[21] = +tmpObjS[109];
tmpR2[22] = +tmpObjS[110];
tmpR2[23] = +tmpObjS[111];
tmpR2[24] = +tmpObjS[112];
tmpR2[25] = +tmpObjS[113];
tmpR2[26] = +tmpObjS[114];
tmpR2[27] = +tmpObjS[115];
tmpR2[28] = +tmpObjS[116];
tmpR2[29] = +tmpObjS[117];
tmpR2[30] = +tmpObjS[118];
tmpR2[31] = +tmpObjS[119];
tmpR2[32] = +tmpObjS[120];
tmpR1[0] = + tmpR2[8];
tmpR1[1] = + tmpR2[9];
tmpR1[2] = + tmpR2[10];
tmpR1[3] = + tmpR2[19];
tmpR1[4] = + tmpR2[20];
tmpR1[5] = + tmpR2[21];
tmpR1[6] = + tmpR2[30];
tmpR1[7] = + tmpR2[31];
tmpR1[8] = + tmpR2[32];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[18];
tmpQN2[1] = +tmpObjSEndTerm[19];
tmpQN2[2] = +tmpObjSEndTerm[20];
tmpQN2[3] = +tmpObjSEndTerm[21];
tmpQN2[4] = +tmpObjSEndTerm[22];
tmpQN2[5] = +tmpObjSEndTerm[23];
tmpQN2[6] = +tmpObjSEndTerm[24];
tmpQN2[7] = +tmpObjSEndTerm[25];
tmpQN2[8] = +tmpObjSEndTerm[26];
tmpQN2[9] = +tmpObjSEndTerm[27];
tmpQN2[10] = +tmpObjSEndTerm[28];
tmpQN2[11] = +tmpObjSEndTerm[29];
tmpQN2[12] = +tmpObjSEndTerm[30];
tmpQN2[13] = +tmpObjSEndTerm[31];
tmpQN2[14] = +tmpObjSEndTerm[32];
tmpQN2[15] = +tmpObjSEndTerm[33];
tmpQN2[16] = +tmpObjSEndTerm[34];
tmpQN2[17] = +tmpObjSEndTerm[35];
tmpQN2[18] = 0.0;
;
tmpQN2[19] = 0.0;
;
tmpQN2[20] = 0.0;
;
tmpQN2[21] = 0.0;
;
tmpQN2[22] = 0.0;
;
tmpQN2[23] = 0.0;
;
tmpQN2[24] = 0.0;
;
tmpQN2[25] = 0.0;
;
tmpQN2[26] = 0.0;
;
tmpQN2[27] = 0.0;
;
tmpQN2[28] = 0.0;
;
tmpQN2[29] = 0.0;
;
tmpQN2[30] = 0.0;
;
tmpQN2[31] = 0.0;
;
tmpQN2[32] = 0.0;
;
tmpQN2[33] = 0.0;
;
tmpQN2[34] = 0.0;
;
tmpQN2[35] = 0.0;
;
tmpQN2[36] = +tmpObjSEndTerm[0];
tmpQN2[37] = +tmpObjSEndTerm[1];
tmpQN2[38] = +tmpObjSEndTerm[2];
tmpQN2[39] = +tmpObjSEndTerm[3];
tmpQN2[40] = +tmpObjSEndTerm[4];
tmpQN2[41] = +tmpObjSEndTerm[5];
tmpQN2[42] = +tmpObjSEndTerm[6];
tmpQN2[43] = +tmpObjSEndTerm[7];
tmpQN2[44] = +tmpObjSEndTerm[8];
tmpQN2[45] = +tmpObjSEndTerm[9];
tmpQN2[46] = +tmpObjSEndTerm[10];
tmpQN2[47] = +tmpObjSEndTerm[11];
tmpQN2[48] = +tmpObjSEndTerm[12];
tmpQN2[49] = +tmpObjSEndTerm[13];
tmpQN2[50] = +tmpObjSEndTerm[14];
tmpQN2[51] = +tmpObjSEndTerm[15];
tmpQN2[52] = +tmpObjSEndTerm[16];
tmpQN2[53] = +tmpObjSEndTerm[17];
tmpQN1[0] = + tmpQN2[3];
tmpQN1[1] = + tmpQN2[4];
tmpQN1[2] = + tmpQN2[5];
tmpQN1[3] = 0.0;
;
tmpQN1[4] = 0.0;
;
tmpQN1[5] = 0.0;
;
tmpQN1[6] = + tmpQN2[0];
tmpQN1[7] = + tmpQN2[1];
tmpQN1[8] = + tmpQN2[2];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = 0.0;
;
tmpQN1[13] = 0.0;
;
tmpQN1[14] = 0.0;
;
tmpQN1[15] = + tmpQN2[6];
tmpQN1[16] = + tmpQN2[7];
tmpQN1[17] = + tmpQN2[8];
tmpQN1[18] = + tmpQN2[15];
tmpQN1[19] = + tmpQN2[16];
tmpQN1[20] = + tmpQN2[17];
tmpQN1[21] = 0.0;
;
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = + tmpQN2[12];
tmpQN1[25] = + tmpQN2[13];
tmpQN1[26] = + tmpQN2[14];
tmpQN1[27] = + tmpQN2[21];
tmpQN1[28] = + tmpQN2[22];
tmpQN1[29] = + tmpQN2[23];
tmpQN1[30] = 0.0;
;
tmpQN1[31] = 0.0;
;
tmpQN1[32] = 0.0;
;
tmpQN1[33] = + tmpQN2[18];
tmpQN1[34] = + tmpQN2[19];
tmpQN1[35] = + tmpQN2[20];
tmpQN1[36] = + tmpQN2[27];
tmpQN1[37] = + tmpQN2[28];
tmpQN1[38] = + tmpQN2[29];
tmpQN1[39] = 0.0;
;
tmpQN1[40] = 0.0;
;
tmpQN1[41] = 0.0;
;
tmpQN1[42] = + tmpQN2[24];
tmpQN1[43] = + tmpQN2[25];
tmpQN1[44] = + tmpQN2[26];
tmpQN1[45] = + tmpQN2[33];
tmpQN1[46] = + tmpQN2[34];
tmpQN1[47] = + tmpQN2[35];
tmpQN1[48] = 0.0;
;
tmpQN1[49] = 0.0;
;
tmpQN1[50] = 0.0;
;
tmpQN1[51] = + tmpQN2[30];
tmpQN1[52] = + tmpQN2[31];
tmpQN1[53] = + tmpQN2[32];
tmpQN1[54] = + tmpQN2[39];
tmpQN1[55] = + tmpQN2[40];
tmpQN1[56] = + tmpQN2[41];
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[36];
tmpQN1[61] = + tmpQN2[37];
tmpQN1[62] = + tmpQN2[38];
tmpQN1[63] = + tmpQN2[45];
tmpQN1[64] = + tmpQN2[46];
tmpQN1[65] = + tmpQN2[47];
tmpQN1[66] = 0.0;
;
tmpQN1[67] = 0.0;
;
tmpQN1[68] = 0.0;
;
tmpQN1[69] = + tmpQN2[42];
tmpQN1[70] = + tmpQN2[43];
tmpQN1[71] = + tmpQN2[44];
tmpQN1[72] = + tmpQN2[51];
tmpQN1[73] = + tmpQN2[52];
tmpQN1[74] = + tmpQN2[53];
tmpQN1[75] = 0.0;
;
tmpQN1[76] = 0.0;
;
tmpQN1[77] = 0.0;
;
tmpQN1[78] = + tmpQN2[48];
tmpQN1[79] = + tmpQN2[49];
tmpQN1[80] = + tmpQN2[50];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 40; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 9];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 9 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 9 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 9 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 9 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 9 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 9 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 9 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 11] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 11 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 11 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 11 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 11 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 11 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 11 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 11 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 11 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 11 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 11 + 10] = acadoWorkspace.objValueOut[10];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 81 ]), &(acadoWorkspace.Q2[ runObj * 99 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 33 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[360];
acadoWorkspace.objValueIn[1] = acadoVariables.x[361];
acadoWorkspace.objValueIn[2] = acadoVariables.x[362];
acadoWorkspace.objValueIn[3] = acadoVariables.x[363];
acadoWorkspace.objValueIn[4] = acadoVariables.x[364];
acadoWorkspace.objValueIn[5] = acadoVariables.x[365];
acadoWorkspace.objValueIn[6] = acadoVariables.x[366];
acadoWorkspace.objValueIn[7] = acadoVariables.x[367];
acadoWorkspace.objValueIn[8] = acadoVariables.x[368];
acadoWorkspace.objValueIn[9] = acadoVariables.od[360];
acadoWorkspace.objValueIn[10] = acadoVariables.od[361];
acadoWorkspace.objValueIn[11] = acadoVariables.od[362];
acadoWorkspace.objValueIn[12] = acadoVariables.od[363];
acadoWorkspace.objValueIn[13] = acadoVariables.od[364];
acadoWorkspace.objValueIn[14] = acadoVariables.od[365];
acadoWorkspace.objValueIn[15] = acadoVariables.od[366];
acadoWorkspace.objValueIn[16] = acadoVariables.od[367];
acadoWorkspace.objValueIn[17] = acadoVariables.od[368];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26];
Gu2[3] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[15] + Gx1[15]*Gu1[18] + Gx1[16]*Gu1[21] + Gx1[17]*Gu1[24];
Gu2[4] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[19] + Gx1[16]*Gu1[22] + Gx1[17]*Gu1[25];
Gu2[5] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[23] + Gx1[17]*Gu1[26];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[24];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[25];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17] + Gx1[24]*Gu1[20] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[26];
Gu2[9] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[31]*Gu1[12] + Gx1[32]*Gu1[15] + Gx1[33]*Gu1[18] + Gx1[34]*Gu1[21] + Gx1[35]*Gu1[24];
Gu2[10] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[31]*Gu1[13] + Gx1[32]*Gu1[16] + Gx1[33]*Gu1[19] + Gx1[34]*Gu1[22] + Gx1[35]*Gu1[25];
Gu2[11] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[31]*Gu1[14] + Gx1[32]*Gu1[17] + Gx1[33]*Gu1[20] + Gx1[34]*Gu1[23] + Gx1[35]*Gu1[26];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26];
Gu2[15] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[3] + Gx1[47]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[51]*Gu1[18] + Gx1[52]*Gu1[21] + Gx1[53]*Gu1[24];
Gu2[16] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[51]*Gu1[19] + Gx1[52]*Gu1[22] + Gx1[53]*Gu1[25];
Gu2[17] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[51]*Gu1[20] + Gx1[52]*Gu1[23] + Gx1[53]*Gu1[26];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[3] + Gx1[56]*Gu1[6] + Gx1[57]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[24];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[7] + Gx1[57]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[59]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[25];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[59]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[26];
Gu2[21] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[24];
Gu2[22] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[25];
Gu2[23] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[26];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 360) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 360) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 360) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 363] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + R11[0];
acadoWorkspace.H[iRow * 363 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + R11[1];
acadoWorkspace.H[iRow * 363 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + R11[2];
acadoWorkspace.H[iRow * 363 + 120] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + R11[3];
acadoWorkspace.H[iRow * 363 + 121] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + R11[4];
acadoWorkspace.H[iRow * 363 + 122] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + R11[5];
acadoWorkspace.H[iRow * 363 + 240] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + R11[6];
acadoWorkspace.H[iRow * 363 + 241] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + R11[7];
acadoWorkspace.H[iRow * 363 + 242] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + R11[8];
acadoWorkspace.H[iRow * 363] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 363 + 121] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 363 + 242] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[18]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[36]*Gu1[12] + Gx1[45]*Gu1[15] + Gx1[54]*Gu1[18] + Gx1[63]*Gu1[21] + Gx1[72]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[18]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[36]*Gu1[13] + Gx1[45]*Gu1[16] + Gx1[54]*Gu1[19] + Gx1[63]*Gu1[22] + Gx1[72]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[18]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[36]*Gu1[14] + Gx1[45]*Gu1[17] + Gx1[54]*Gu1[20] + Gx1[63]*Gu1[23] + Gx1[72]*Gu1[26];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[19]*Gu1[6] + Gx1[28]*Gu1[9] + Gx1[37]*Gu1[12] + Gx1[46]*Gu1[15] + Gx1[55]*Gu1[18] + Gx1[64]*Gu1[21] + Gx1[73]*Gu1[24];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[19]*Gu1[7] + Gx1[28]*Gu1[10] + Gx1[37]*Gu1[13] + Gx1[46]*Gu1[16] + Gx1[55]*Gu1[19] + Gx1[64]*Gu1[22] + Gx1[73]*Gu1[25];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[19]*Gu1[8] + Gx1[28]*Gu1[11] + Gx1[37]*Gu1[14] + Gx1[46]*Gu1[17] + Gx1[55]*Gu1[20] + Gx1[64]*Gu1[23] + Gx1[73]*Gu1[26];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[29]*Gu1[9] + Gx1[38]*Gu1[12] + Gx1[47]*Gu1[15] + Gx1[56]*Gu1[18] + Gx1[65]*Gu1[21] + Gx1[74]*Gu1[24];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[29]*Gu1[10] + Gx1[38]*Gu1[13] + Gx1[47]*Gu1[16] + Gx1[56]*Gu1[19] + Gx1[65]*Gu1[22] + Gx1[74]*Gu1[25];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[29]*Gu1[11] + Gx1[38]*Gu1[14] + Gx1[47]*Gu1[17] + Gx1[56]*Gu1[20] + Gx1[65]*Gu1[23] + Gx1[74]*Gu1[26];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[12]*Gu1[3] + Gx1[21]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[39]*Gu1[12] + Gx1[48]*Gu1[15] + Gx1[57]*Gu1[18] + Gx1[66]*Gu1[21] + Gx1[75]*Gu1[24];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[12]*Gu1[4] + Gx1[21]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[39]*Gu1[13] + Gx1[48]*Gu1[16] + Gx1[57]*Gu1[19] + Gx1[66]*Gu1[22] + Gx1[75]*Gu1[25];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[12]*Gu1[5] + Gx1[21]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[39]*Gu1[14] + Gx1[48]*Gu1[17] + Gx1[57]*Gu1[20] + Gx1[66]*Gu1[23] + Gx1[75]*Gu1[26];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[22]*Gu1[6] + Gx1[31]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[49]*Gu1[15] + Gx1[58]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[76]*Gu1[24];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[22]*Gu1[7] + Gx1[31]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[49]*Gu1[16] + Gx1[58]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[76]*Gu1[25];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[22]*Gu1[8] + Gx1[31]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[49]*Gu1[17] + Gx1[58]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[76]*Gu1[26];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[23]*Gu1[6] + Gx1[32]*Gu1[9] + Gx1[41]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[59]*Gu1[18] + Gx1[68]*Gu1[21] + Gx1[77]*Gu1[24];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[23]*Gu1[7] + Gx1[32]*Gu1[10] + Gx1[41]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[59]*Gu1[19] + Gx1[68]*Gu1[22] + Gx1[77]*Gu1[25];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[23]*Gu1[8] + Gx1[32]*Gu1[11] + Gx1[41]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[59]*Gu1[20] + Gx1[68]*Gu1[23] + Gx1[77]*Gu1[26];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[24]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[42]*Gu1[12] + Gx1[51]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[69]*Gu1[21] + Gx1[78]*Gu1[24];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[24]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[42]*Gu1[13] + Gx1[51]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[69]*Gu1[22] + Gx1[78]*Gu1[25];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[24]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[42]*Gu1[14] + Gx1[51]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[69]*Gu1[23] + Gx1[78]*Gu1[26];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[25]*Gu1[6] + Gx1[34]*Gu1[9] + Gx1[43]*Gu1[12] + Gx1[52]*Gu1[15] + Gx1[61]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[79]*Gu1[24];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[25]*Gu1[7] + Gx1[34]*Gu1[10] + Gx1[43]*Gu1[13] + Gx1[52]*Gu1[16] + Gx1[61]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[79]*Gu1[25];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[25]*Gu1[8] + Gx1[34]*Gu1[11] + Gx1[43]*Gu1[14] + Gx1[52]*Gu1[17] + Gx1[61]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[79]*Gu1[26];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[35]*Gu1[9] + Gx1[44]*Gu1[12] + Gx1[53]*Gu1[15] + Gx1[62]*Gu1[18] + Gx1[71]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[35]*Gu1[10] + Gx1[44]*Gu1[13] + Gx1[53]*Gu1[16] + Gx1[62]*Gu1[19] + Gx1[71]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[35]*Gu1[11] + Gx1[44]*Gu1[14] + Gx1[53]*Gu1[17] + Gx1[62]*Gu1[20] + Gx1[71]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Gu2[2];
Gu3[3] = + Q11[9]*Gu1[0] + Q11[10]*Gu1[3] + Q11[11]*Gu1[6] + Q11[12]*Gu1[9] + Q11[13]*Gu1[12] + Q11[14]*Gu1[15] + Q11[15]*Gu1[18] + Q11[16]*Gu1[21] + Q11[17]*Gu1[24] + Gu2[3];
Gu3[4] = + Q11[9]*Gu1[1] + Q11[10]*Gu1[4] + Q11[11]*Gu1[7] + Q11[12]*Gu1[10] + Q11[13]*Gu1[13] + Q11[14]*Gu1[16] + Q11[15]*Gu1[19] + Q11[16]*Gu1[22] + Q11[17]*Gu1[25] + Gu2[4];
Gu3[5] = + Q11[9]*Gu1[2] + Q11[10]*Gu1[5] + Q11[11]*Gu1[8] + Q11[12]*Gu1[11] + Q11[13]*Gu1[14] + Q11[14]*Gu1[17] + Q11[15]*Gu1[20] + Q11[16]*Gu1[23] + Q11[17]*Gu1[26] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[3] + Q11[20]*Gu1[6] + Q11[21]*Gu1[9] + Q11[22]*Gu1[12] + Q11[23]*Gu1[15] + Q11[24]*Gu1[18] + Q11[25]*Gu1[21] + Q11[26]*Gu1[24] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[4] + Q11[20]*Gu1[7] + Q11[21]*Gu1[10] + Q11[22]*Gu1[13] + Q11[23]*Gu1[16] + Q11[24]*Gu1[19] + Q11[25]*Gu1[22] + Q11[26]*Gu1[25] + Gu2[7];
Gu3[8] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[5] + Q11[20]*Gu1[8] + Q11[21]*Gu1[11] + Q11[22]*Gu1[14] + Q11[23]*Gu1[17] + Q11[24]*Gu1[20] + Q11[25]*Gu1[23] + Q11[26]*Gu1[26] + Gu2[8];
Gu3[9] = + Q11[27]*Gu1[0] + Q11[28]*Gu1[3] + Q11[29]*Gu1[6] + Q11[30]*Gu1[9] + Q11[31]*Gu1[12] + Q11[32]*Gu1[15] + Q11[33]*Gu1[18] + Q11[34]*Gu1[21] + Q11[35]*Gu1[24] + Gu2[9];
Gu3[10] = + Q11[27]*Gu1[1] + Q11[28]*Gu1[4] + Q11[29]*Gu1[7] + Q11[30]*Gu1[10] + Q11[31]*Gu1[13] + Q11[32]*Gu1[16] + Q11[33]*Gu1[19] + Q11[34]*Gu1[22] + Q11[35]*Gu1[25] + Gu2[10];
Gu3[11] = + Q11[27]*Gu1[2] + Q11[28]*Gu1[5] + Q11[29]*Gu1[8] + Q11[30]*Gu1[11] + Q11[31]*Gu1[14] + Q11[32]*Gu1[17] + Q11[33]*Gu1[20] + Q11[34]*Gu1[23] + Q11[35]*Gu1[26] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[3] + Q11[38]*Gu1[6] + Q11[39]*Gu1[9] + Q11[40]*Gu1[12] + Q11[41]*Gu1[15] + Q11[42]*Gu1[18] + Q11[43]*Gu1[21] + Q11[44]*Gu1[24] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[4] + Q11[38]*Gu1[7] + Q11[39]*Gu1[10] + Q11[40]*Gu1[13] + Q11[41]*Gu1[16] + Q11[42]*Gu1[19] + Q11[43]*Gu1[22] + Q11[44]*Gu1[25] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[5] + Q11[38]*Gu1[8] + Q11[39]*Gu1[11] + Q11[40]*Gu1[14] + Q11[41]*Gu1[17] + Q11[42]*Gu1[20] + Q11[43]*Gu1[23] + Q11[44]*Gu1[26] + Gu2[14];
Gu3[15] = + Q11[45]*Gu1[0] + Q11[46]*Gu1[3] + Q11[47]*Gu1[6] + Q11[48]*Gu1[9] + Q11[49]*Gu1[12] + Q11[50]*Gu1[15] + Q11[51]*Gu1[18] + Q11[52]*Gu1[21] + Q11[53]*Gu1[24] + Gu2[15];
Gu3[16] = + Q11[45]*Gu1[1] + Q11[46]*Gu1[4] + Q11[47]*Gu1[7] + Q11[48]*Gu1[10] + Q11[49]*Gu1[13] + Q11[50]*Gu1[16] + Q11[51]*Gu1[19] + Q11[52]*Gu1[22] + Q11[53]*Gu1[25] + Gu2[16];
Gu3[17] = + Q11[45]*Gu1[2] + Q11[46]*Gu1[5] + Q11[47]*Gu1[8] + Q11[48]*Gu1[11] + Q11[49]*Gu1[14] + Q11[50]*Gu1[17] + Q11[51]*Gu1[20] + Q11[52]*Gu1[23] + Q11[53]*Gu1[26] + Gu2[17];
Gu3[18] = + Q11[54]*Gu1[0] + Q11[55]*Gu1[3] + Q11[56]*Gu1[6] + Q11[57]*Gu1[9] + Q11[58]*Gu1[12] + Q11[59]*Gu1[15] + Q11[60]*Gu1[18] + Q11[61]*Gu1[21] + Q11[62]*Gu1[24] + Gu2[18];
Gu3[19] = + Q11[54]*Gu1[1] + Q11[55]*Gu1[4] + Q11[56]*Gu1[7] + Q11[57]*Gu1[10] + Q11[58]*Gu1[13] + Q11[59]*Gu1[16] + Q11[60]*Gu1[19] + Q11[61]*Gu1[22] + Q11[62]*Gu1[25] + Gu2[19];
Gu3[20] = + Q11[54]*Gu1[2] + Q11[55]*Gu1[5] + Q11[56]*Gu1[8] + Q11[57]*Gu1[11] + Q11[58]*Gu1[14] + Q11[59]*Gu1[17] + Q11[60]*Gu1[20] + Q11[61]*Gu1[23] + Q11[62]*Gu1[26] + Gu2[20];
Gu3[21] = + Q11[63]*Gu1[0] + Q11[64]*Gu1[3] + Q11[65]*Gu1[6] + Q11[66]*Gu1[9] + Q11[67]*Gu1[12] + Q11[68]*Gu1[15] + Q11[69]*Gu1[18] + Q11[70]*Gu1[21] + Q11[71]*Gu1[24] + Gu2[21];
Gu3[22] = + Q11[63]*Gu1[1] + Q11[64]*Gu1[4] + Q11[65]*Gu1[7] + Q11[66]*Gu1[10] + Q11[67]*Gu1[13] + Q11[68]*Gu1[16] + Q11[69]*Gu1[19] + Q11[70]*Gu1[22] + Q11[71]*Gu1[25] + Gu2[22];
Gu3[23] = + Q11[63]*Gu1[2] + Q11[64]*Gu1[5] + Q11[65]*Gu1[8] + Q11[66]*Gu1[11] + Q11[67]*Gu1[14] + Q11[68]*Gu1[17] + Q11[69]*Gu1[20] + Q11[70]*Gu1[23] + Q11[71]*Gu1[26] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[3] + Q11[74]*Gu1[6] + Q11[75]*Gu1[9] + Q11[76]*Gu1[12] + Q11[77]*Gu1[15] + Q11[78]*Gu1[18] + Q11[79]*Gu1[21] + Q11[80]*Gu1[24] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[4] + Q11[74]*Gu1[7] + Q11[75]*Gu1[10] + Q11[76]*Gu1[13] + Q11[77]*Gu1[16] + Q11[78]*Gu1[19] + Q11[79]*Gu1[22] + Q11[80]*Gu1[25] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[5] + Q11[74]*Gu1[8] + Q11[75]*Gu1[11] + Q11[76]*Gu1[14] + Q11[77]*Gu1[17] + Q11[78]*Gu1[20] + Q11[79]*Gu1[23] + Q11[80]*Gu1[26] + Gu2[26];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[9]*w11[1] + Gx1[18]*w11[2] + Gx1[27]*w11[3] + Gx1[36]*w11[4] + Gx1[45]*w11[5] + Gx1[54]*w11[6] + Gx1[63]*w11[7] + Gx1[72]*w11[8] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[10]*w11[1] + Gx1[19]*w11[2] + Gx1[28]*w11[3] + Gx1[37]*w11[4] + Gx1[46]*w11[5] + Gx1[55]*w11[6] + Gx1[64]*w11[7] + Gx1[73]*w11[8] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[11]*w11[1] + Gx1[20]*w11[2] + Gx1[29]*w11[3] + Gx1[38]*w11[4] + Gx1[47]*w11[5] + Gx1[56]*w11[6] + Gx1[65]*w11[7] + Gx1[74]*w11[8] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[12]*w11[1] + Gx1[21]*w11[2] + Gx1[30]*w11[3] + Gx1[39]*w11[4] + Gx1[48]*w11[5] + Gx1[57]*w11[6] + Gx1[66]*w11[7] + Gx1[75]*w11[8] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[13]*w11[1] + Gx1[22]*w11[2] + Gx1[31]*w11[3] + Gx1[40]*w11[4] + Gx1[49]*w11[5] + Gx1[58]*w11[6] + Gx1[67]*w11[7] + Gx1[76]*w11[8] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[14]*w11[1] + Gx1[23]*w11[2] + Gx1[32]*w11[3] + Gx1[41]*w11[4] + Gx1[50]*w11[5] + Gx1[59]*w11[6] + Gx1[68]*w11[7] + Gx1[77]*w11[8] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[15]*w11[1] + Gx1[24]*w11[2] + Gx1[33]*w11[3] + Gx1[42]*w11[4] + Gx1[51]*w11[5] + Gx1[60]*w11[6] + Gx1[69]*w11[7] + Gx1[78]*w11[8] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[16]*w11[1] + Gx1[25]*w11[2] + Gx1[34]*w11[3] + Gx1[43]*w11[4] + Gx1[52]*w11[5] + Gx1[61]*w11[6] + Gx1[70]*w11[7] + Gx1[79]*w11[8] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[17]*w11[1] + Gx1[26]*w11[2] + Gx1[35]*w11[3] + Gx1[44]*w11[4] + Gx1[53]*w11[5] + Gx1[62]*w11[6] + Gx1[71]*w11[7] + Gx1[80]*w11[8] + w12[8];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + w12[0];
w13[1] = + Q11[9]*w11[0] + Q11[10]*w11[1] + Q11[11]*w11[2] + Q11[12]*w11[3] + Q11[13]*w11[4] + Q11[14]*w11[5] + Q11[15]*w11[6] + Q11[16]*w11[7] + Q11[17]*w11[8] + w12[1];
w13[2] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + Q11[24]*w11[6] + Q11[25]*w11[7] + Q11[26]*w11[8] + w12[2];
w13[3] = + Q11[27]*w11[0] + Q11[28]*w11[1] + Q11[29]*w11[2] + Q11[30]*w11[3] + Q11[31]*w11[4] + Q11[32]*w11[5] + Q11[33]*w11[6] + Q11[34]*w11[7] + Q11[35]*w11[8] + w12[3];
w13[4] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + w12[4];
w13[5] = + Q11[45]*w11[0] + Q11[46]*w11[1] + Q11[47]*w11[2] + Q11[48]*w11[3] + Q11[49]*w11[4] + Q11[50]*w11[5] + Q11[51]*w11[6] + Q11[52]*w11[7] + Q11[53]*w11[8] + w12[5];
w13[6] = + Q11[54]*w11[0] + Q11[55]*w11[1] + Q11[56]*w11[2] + Q11[57]*w11[3] + Q11[58]*w11[4] + Q11[59]*w11[5] + Q11[60]*w11[6] + Q11[61]*w11[7] + Q11[62]*w11[8] + w12[6];
w13[7] = + Q11[63]*w11[0] + Q11[64]*w11[1] + Q11[65]*w11[2] + Q11[66]*w11[3] + Q11[67]*w11[4] + Q11[68]*w11[5] + Q11[69]*w11[6] + Q11[70]*w11[7] + Q11[71]*w11[8] + w12[7];
w13[8] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + w12[8];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 360) + (iCol * 3)] = acadoWorkspace.H[(iCol * 360) + (iRow * 3)];
acadoWorkspace.H[(iRow * 360) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 3)];
acadoWorkspace.H[(iRow * 360) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 3)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 3)] = acadoWorkspace.H[(iCol * 360) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 3)] = acadoWorkspace.H[(iCol * 360) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10];
RDy1[1] = + R2[11]*Dy1[0] + R2[12]*Dy1[1] + R2[13]*Dy1[2] + R2[14]*Dy1[3] + R2[15]*Dy1[4] + R2[16]*Dy1[5] + R2[17]*Dy1[6] + R2[18]*Dy1[7] + R2[19]*Dy1[8] + R2[20]*Dy1[9] + R2[21]*Dy1[10];
RDy1[2] = + R2[22]*Dy1[0] + R2[23]*Dy1[1] + R2[24]*Dy1[2] + R2[25]*Dy1[3] + R2[26]*Dy1[4] + R2[27]*Dy1[5] + R2[28]*Dy1[6] + R2[29]*Dy1[7] + R2[30]*Dy1[8] + R2[31]*Dy1[9] + R2[32]*Dy1[10];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10];
QDy1[1] = + Q2[11]*Dy1[0] + Q2[12]*Dy1[1] + Q2[13]*Dy1[2] + Q2[14]*Dy1[3] + Q2[15]*Dy1[4] + Q2[16]*Dy1[5] + Q2[17]*Dy1[6] + Q2[18]*Dy1[7] + Q2[19]*Dy1[8] + Q2[20]*Dy1[9] + Q2[21]*Dy1[10];
QDy1[2] = + Q2[22]*Dy1[0] + Q2[23]*Dy1[1] + Q2[24]*Dy1[2] + Q2[25]*Dy1[3] + Q2[26]*Dy1[4] + Q2[27]*Dy1[5] + Q2[28]*Dy1[6] + Q2[29]*Dy1[7] + Q2[30]*Dy1[8] + Q2[31]*Dy1[9] + Q2[32]*Dy1[10];
QDy1[3] = + Q2[33]*Dy1[0] + Q2[34]*Dy1[1] + Q2[35]*Dy1[2] + Q2[36]*Dy1[3] + Q2[37]*Dy1[4] + Q2[38]*Dy1[5] + Q2[39]*Dy1[6] + Q2[40]*Dy1[7] + Q2[41]*Dy1[8] + Q2[42]*Dy1[9] + Q2[43]*Dy1[10];
QDy1[4] = + Q2[44]*Dy1[0] + Q2[45]*Dy1[1] + Q2[46]*Dy1[2] + Q2[47]*Dy1[3] + Q2[48]*Dy1[4] + Q2[49]*Dy1[5] + Q2[50]*Dy1[6] + Q2[51]*Dy1[7] + Q2[52]*Dy1[8] + Q2[53]*Dy1[9] + Q2[54]*Dy1[10];
QDy1[5] = + Q2[55]*Dy1[0] + Q2[56]*Dy1[1] + Q2[57]*Dy1[2] + Q2[58]*Dy1[3] + Q2[59]*Dy1[4] + Q2[60]*Dy1[5] + Q2[61]*Dy1[6] + Q2[62]*Dy1[7] + Q2[63]*Dy1[8] + Q2[64]*Dy1[9] + Q2[65]*Dy1[10];
QDy1[6] = + Q2[66]*Dy1[0] + Q2[67]*Dy1[1] + Q2[68]*Dy1[2] + Q2[69]*Dy1[3] + Q2[70]*Dy1[4] + Q2[71]*Dy1[5] + Q2[72]*Dy1[6] + Q2[73]*Dy1[7] + Q2[74]*Dy1[8] + Q2[75]*Dy1[9] + Q2[76]*Dy1[10];
QDy1[7] = + Q2[77]*Dy1[0] + Q2[78]*Dy1[1] + Q2[79]*Dy1[2] + Q2[80]*Dy1[3] + Q2[81]*Dy1[4] + Q2[82]*Dy1[5] + Q2[83]*Dy1[6] + Q2[84]*Dy1[7] + Q2[85]*Dy1[8] + Q2[86]*Dy1[9] + Q2[87]*Dy1[10];
QDy1[8] = + Q2[88]*Dy1[0] + Q2[89]*Dy1[1] + Q2[90]*Dy1[2] + Q2[91]*Dy1[3] + Q2[92]*Dy1[4] + Q2[93]*Dy1[5] + Q2[94]*Dy1[6] + Q2[95]*Dy1[7] + Q2[96]*Dy1[8] + Q2[97]*Dy1[9] + Q2[98]*Dy1[10];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 40; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 81)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 27 ]), &(acadoWorkspace.E[ lRun3 * 27 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 40; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (9)) * (9)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (9)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (9)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (40)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 39; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 27 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 81 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 81 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 27 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 360; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];


}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
for (lRun1 = 0; lRun1 < 440; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 33 ]), &(acadoWorkspace.Dy[ 11 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 66 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 99 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 132 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 165 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 198 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 231 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 297 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 330 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 363 ]), &(acadoWorkspace.Dy[ 121 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 429 ]), &(acadoWorkspace.Dy[ 143 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 462 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 495 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 528 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 561 ]), &(acadoWorkspace.Dy[ 187 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 594 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 627 ]), &(acadoWorkspace.Dy[ 209 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 660 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 693 ]), &(acadoWorkspace.Dy[ 231 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 726 ]), &(acadoWorkspace.Dy[ 242 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 759 ]), &(acadoWorkspace.Dy[ 253 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 792 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 825 ]), &(acadoWorkspace.Dy[ 275 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 858 ]), &(acadoWorkspace.Dy[ 286 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 891 ]), &(acadoWorkspace.Dy[ 297 ]), &(acadoWorkspace.g[ 81 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 924 ]), &(acadoWorkspace.Dy[ 308 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 957 ]), &(acadoWorkspace.Dy[ 319 ]), &(acadoWorkspace.g[ 87 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 990 ]), &(acadoWorkspace.Dy[ 330 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1023 ]), &(acadoWorkspace.Dy[ 341 ]), &(acadoWorkspace.g[ 93 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1056 ]), &(acadoWorkspace.Dy[ 352 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1089 ]), &(acadoWorkspace.Dy[ 363 ]), &(acadoWorkspace.g[ 99 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1122 ]), &(acadoWorkspace.Dy[ 374 ]), &(acadoWorkspace.g[ 102 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1155 ]), &(acadoWorkspace.Dy[ 385 ]), &(acadoWorkspace.g[ 105 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1188 ]), &(acadoWorkspace.Dy[ 396 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1221 ]), &(acadoWorkspace.Dy[ 407 ]), &(acadoWorkspace.g[ 111 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1254 ]), &(acadoWorkspace.Dy[ 418 ]), &(acadoWorkspace.g[ 114 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1287 ]), &(acadoWorkspace.Dy[ 429 ]), &(acadoWorkspace.g[ 117 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 99 ]), &(acadoWorkspace.Dy[ 11 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 198 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 297 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 396 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 495 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 594 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 693 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 792 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 891 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 990 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1089 ]), &(acadoWorkspace.Dy[ 121 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1188 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1287 ]), &(acadoWorkspace.Dy[ 143 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1386 ]), &(acadoWorkspace.Dy[ 154 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1485 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1584 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1683 ]), &(acadoWorkspace.Dy[ 187 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1782 ]), &(acadoWorkspace.Dy[ 198 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1881 ]), &(acadoWorkspace.Dy[ 209 ]), &(acadoWorkspace.QDy[ 171 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1980 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2079 ]), &(acadoWorkspace.Dy[ 231 ]), &(acadoWorkspace.QDy[ 189 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2178 ]), &(acadoWorkspace.Dy[ 242 ]), &(acadoWorkspace.QDy[ 198 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2277 ]), &(acadoWorkspace.Dy[ 253 ]), &(acadoWorkspace.QDy[ 207 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2376 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2475 ]), &(acadoWorkspace.Dy[ 275 ]), &(acadoWorkspace.QDy[ 225 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2574 ]), &(acadoWorkspace.Dy[ 286 ]), &(acadoWorkspace.QDy[ 234 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2673 ]), &(acadoWorkspace.Dy[ 297 ]), &(acadoWorkspace.QDy[ 243 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2772 ]), &(acadoWorkspace.Dy[ 308 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2871 ]), &(acadoWorkspace.Dy[ 319 ]), &(acadoWorkspace.QDy[ 261 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2970 ]), &(acadoWorkspace.Dy[ 330 ]), &(acadoWorkspace.QDy[ 270 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3069 ]), &(acadoWorkspace.Dy[ 341 ]), &(acadoWorkspace.QDy[ 279 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3168 ]), &(acadoWorkspace.Dy[ 352 ]), &(acadoWorkspace.QDy[ 288 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3267 ]), &(acadoWorkspace.Dy[ 363 ]), &(acadoWorkspace.QDy[ 297 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3366 ]), &(acadoWorkspace.Dy[ 374 ]), &(acadoWorkspace.QDy[ 306 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3465 ]), &(acadoWorkspace.Dy[ 385 ]), &(acadoWorkspace.QDy[ 315 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3564 ]), &(acadoWorkspace.Dy[ 396 ]), &(acadoWorkspace.QDy[ 324 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3663 ]), &(acadoWorkspace.Dy[ 407 ]), &(acadoWorkspace.QDy[ 333 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3762 ]), &(acadoWorkspace.Dy[ 418 ]), &(acadoWorkspace.QDy[ 342 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3861 ]), &(acadoWorkspace.Dy[ 429 ]), &(acadoWorkspace.QDy[ 351 ]) );

acadoWorkspace.QDy[360] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[361] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[362] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[363] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[364] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[365] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[366] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[367] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[368] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1701 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1782 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1863 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2025 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2106 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 243 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2187 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 261 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2349 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.sbar[ 270 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2430 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.sbar[ 279 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2511 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 297 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2673 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.sbar[ 306 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2754 ]), &(acadoWorkspace.sbar[ 306 ]), &(acadoWorkspace.sbar[ 315 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2835 ]), &(acadoWorkspace.sbar[ 315 ]), &(acadoWorkspace.sbar[ 324 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.sbar[ 333 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2997 ]), &(acadoWorkspace.sbar[ 333 ]), &(acadoWorkspace.sbar[ 342 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3078 ]), &(acadoWorkspace.sbar[ 342 ]), &(acadoWorkspace.sbar[ 351 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3159 ]), &(acadoWorkspace.sbar[ 351 ]), &(acadoWorkspace.sbar[ 360 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[360];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[361];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[362];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[363];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[364];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[365];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[366];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[367];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[360] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[361] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[362] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[363] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[364] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[365] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[366] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[367] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[368] + acadoWorkspace.QDy[368];
acado_macBTw1( &(acadoWorkspace.evGu[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 117 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3159 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 351 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3159 ]), &(acadoWorkspace.sbar[ 351 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1026 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 114 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3078 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 342 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3078 ]), &(acadoWorkspace.sbar[ 342 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 999 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 111 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2997 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 333 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2997 ]), &(acadoWorkspace.sbar[ 333 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 324 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.sbar[ 324 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 945 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 105 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2835 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 315 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2835 ]), &(acadoWorkspace.sbar[ 315 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 918 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 102 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2754 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 306 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2754 ]), &(acadoWorkspace.sbar[ 306 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 99 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2673 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 297 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2673 ]), &(acadoWorkspace.sbar[ 297 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 288 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.sbar[ 288 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 837 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 93 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2511 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 279 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2511 ]), &(acadoWorkspace.sbar[ 279 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2430 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 270 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2430 ]), &(acadoWorkspace.sbar[ 270 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 783 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 87 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2349 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 261 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2349 ]), &(acadoWorkspace.sbar[ 261 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 252 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.sbar[ 252 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 81 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2187 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 243 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2187 ]), &(acadoWorkspace.sbar[ 243 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 702 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2106 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2106 ]), &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 675 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2025 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 225 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2025 ]), &(acadoWorkspace.sbar[ 225 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 216 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.sbar[ 216 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 621 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1863 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 207 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1863 ]), &(acadoWorkspace.sbar[ 207 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 594 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1782 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 198 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1782 ]), &(acadoWorkspace.sbar[ 198 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1701 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 189 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1701 ]), &(acadoWorkspace.sbar[ 189 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1458 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1377 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1215 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1134 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.lb[80] = acadoVariables.lbValues[80] - acadoVariables.u[80];
acadoWorkspace.lb[81] = acadoVariables.lbValues[81] - acadoVariables.u[81];
acadoWorkspace.lb[82] = acadoVariables.lbValues[82] - acadoVariables.u[82];
acadoWorkspace.lb[83] = acadoVariables.lbValues[83] - acadoVariables.u[83];
acadoWorkspace.lb[84] = acadoVariables.lbValues[84] - acadoVariables.u[84];
acadoWorkspace.lb[85] = acadoVariables.lbValues[85] - acadoVariables.u[85];
acadoWorkspace.lb[86] = acadoVariables.lbValues[86] - acadoVariables.u[86];
acadoWorkspace.lb[87] = acadoVariables.lbValues[87] - acadoVariables.u[87];
acadoWorkspace.lb[88] = acadoVariables.lbValues[88] - acadoVariables.u[88];
acadoWorkspace.lb[89] = acadoVariables.lbValues[89] - acadoVariables.u[89];
acadoWorkspace.lb[90] = acadoVariables.lbValues[90] - acadoVariables.u[90];
acadoWorkspace.lb[91] = acadoVariables.lbValues[91] - acadoVariables.u[91];
acadoWorkspace.lb[92] = acadoVariables.lbValues[92] - acadoVariables.u[92];
acadoWorkspace.lb[93] = acadoVariables.lbValues[93] - acadoVariables.u[93];
acadoWorkspace.lb[94] = acadoVariables.lbValues[94] - acadoVariables.u[94];
acadoWorkspace.lb[95] = acadoVariables.lbValues[95] - acadoVariables.u[95];
acadoWorkspace.lb[96] = acadoVariables.lbValues[96] - acadoVariables.u[96];
acadoWorkspace.lb[97] = acadoVariables.lbValues[97] - acadoVariables.u[97];
acadoWorkspace.lb[98] = acadoVariables.lbValues[98] - acadoVariables.u[98];
acadoWorkspace.lb[99] = acadoVariables.lbValues[99] - acadoVariables.u[99];
acadoWorkspace.lb[100] = acadoVariables.lbValues[100] - acadoVariables.u[100];
acadoWorkspace.lb[101] = acadoVariables.lbValues[101] - acadoVariables.u[101];
acadoWorkspace.lb[102] = acadoVariables.lbValues[102] - acadoVariables.u[102];
acadoWorkspace.lb[103] = acadoVariables.lbValues[103] - acadoVariables.u[103];
acadoWorkspace.lb[104] = acadoVariables.lbValues[104] - acadoVariables.u[104];
acadoWorkspace.lb[105] = acadoVariables.lbValues[105] - acadoVariables.u[105];
acadoWorkspace.lb[106] = acadoVariables.lbValues[106] - acadoVariables.u[106];
acadoWorkspace.lb[107] = acadoVariables.lbValues[107] - acadoVariables.u[107];
acadoWorkspace.lb[108] = acadoVariables.lbValues[108] - acadoVariables.u[108];
acadoWorkspace.lb[109] = acadoVariables.lbValues[109] - acadoVariables.u[109];
acadoWorkspace.lb[110] = acadoVariables.lbValues[110] - acadoVariables.u[110];
acadoWorkspace.lb[111] = acadoVariables.lbValues[111] - acadoVariables.u[111];
acadoWorkspace.lb[112] = acadoVariables.lbValues[112] - acadoVariables.u[112];
acadoWorkspace.lb[113] = acadoVariables.lbValues[113] - acadoVariables.u[113];
acadoWorkspace.lb[114] = acadoVariables.lbValues[114] - acadoVariables.u[114];
acadoWorkspace.lb[115] = acadoVariables.lbValues[115] - acadoVariables.u[115];
acadoWorkspace.lb[116] = acadoVariables.lbValues[116] - acadoVariables.u[116];
acadoWorkspace.lb[117] = acadoVariables.lbValues[117] - acadoVariables.u[117];
acadoWorkspace.lb[118] = acadoVariables.lbValues[118] - acadoVariables.u[118];
acadoWorkspace.lb[119] = acadoVariables.lbValues[119] - acadoVariables.u[119];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[80] = acadoVariables.ubValues[80] - acadoVariables.u[80];
acadoWorkspace.ub[81] = acadoVariables.ubValues[81] - acadoVariables.u[81];
acadoWorkspace.ub[82] = acadoVariables.ubValues[82] - acadoVariables.u[82];
acadoWorkspace.ub[83] = acadoVariables.ubValues[83] - acadoVariables.u[83];
acadoWorkspace.ub[84] = acadoVariables.ubValues[84] - acadoVariables.u[84];
acadoWorkspace.ub[85] = acadoVariables.ubValues[85] - acadoVariables.u[85];
acadoWorkspace.ub[86] = acadoVariables.ubValues[86] - acadoVariables.u[86];
acadoWorkspace.ub[87] = acadoVariables.ubValues[87] - acadoVariables.u[87];
acadoWorkspace.ub[88] = acadoVariables.ubValues[88] - acadoVariables.u[88];
acadoWorkspace.ub[89] = acadoVariables.ubValues[89] - acadoVariables.u[89];
acadoWorkspace.ub[90] = acadoVariables.ubValues[90] - acadoVariables.u[90];
acadoWorkspace.ub[91] = acadoVariables.ubValues[91] - acadoVariables.u[91];
acadoWorkspace.ub[92] = acadoVariables.ubValues[92] - acadoVariables.u[92];
acadoWorkspace.ub[93] = acadoVariables.ubValues[93] - acadoVariables.u[93];
acadoWorkspace.ub[94] = acadoVariables.ubValues[94] - acadoVariables.u[94];
acadoWorkspace.ub[95] = acadoVariables.ubValues[95] - acadoVariables.u[95];
acadoWorkspace.ub[96] = acadoVariables.ubValues[96] - acadoVariables.u[96];
acadoWorkspace.ub[97] = acadoVariables.ubValues[97] - acadoVariables.u[97];
acadoWorkspace.ub[98] = acadoVariables.ubValues[98] - acadoVariables.u[98];
acadoWorkspace.ub[99] = acadoVariables.ubValues[99] - acadoVariables.u[99];
acadoWorkspace.ub[100] = acadoVariables.ubValues[100] - acadoVariables.u[100];
acadoWorkspace.ub[101] = acadoVariables.ubValues[101] - acadoVariables.u[101];
acadoWorkspace.ub[102] = acadoVariables.ubValues[102] - acadoVariables.u[102];
acadoWorkspace.ub[103] = acadoVariables.ubValues[103] - acadoVariables.u[103];
acadoWorkspace.ub[104] = acadoVariables.ubValues[104] - acadoVariables.u[104];
acadoWorkspace.ub[105] = acadoVariables.ubValues[105] - acadoVariables.u[105];
acadoWorkspace.ub[106] = acadoVariables.ubValues[106] - acadoVariables.u[106];
acadoWorkspace.ub[107] = acadoVariables.ubValues[107] - acadoVariables.u[107];
acadoWorkspace.ub[108] = acadoVariables.ubValues[108] - acadoVariables.u[108];
acadoWorkspace.ub[109] = acadoVariables.ubValues[109] - acadoVariables.u[109];
acadoWorkspace.ub[110] = acadoVariables.ubValues[110] - acadoVariables.u[110];
acadoWorkspace.ub[111] = acadoVariables.ubValues[111] - acadoVariables.u[111];
acadoWorkspace.ub[112] = acadoVariables.ubValues[112] - acadoVariables.u[112];
acadoWorkspace.ub[113] = acadoVariables.ubValues[113] - acadoVariables.u[113];
acadoWorkspace.ub[114] = acadoVariables.ubValues[114] - acadoVariables.u[114];
acadoWorkspace.ub[115] = acadoVariables.ubValues[115] - acadoVariables.u[115];
acadoWorkspace.ub[116] = acadoVariables.ubValues[116] - acadoVariables.u[116];
acadoWorkspace.ub[117] = acadoVariables.ubValues[117] - acadoVariables.u[117];
acadoWorkspace.ub[118] = acadoVariables.ubValues[118] - acadoVariables.u[118];
acadoWorkspace.ub[119] = acadoVariables.ubValues[119] - acadoVariables.u[119];

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoVariables.u[100] += acadoWorkspace.x[100];
acadoVariables.u[101] += acadoWorkspace.x[101];
acadoVariables.u[102] += acadoWorkspace.x[102];
acadoVariables.u[103] += acadoWorkspace.x[103];
acadoVariables.u[104] += acadoWorkspace.x[104];
acadoVariables.u[105] += acadoWorkspace.x[105];
acadoVariables.u[106] += acadoWorkspace.x[106];
acadoVariables.u[107] += acadoWorkspace.x[107];
acadoVariables.u[108] += acadoWorkspace.x[108];
acadoVariables.u[109] += acadoWorkspace.x[109];
acadoVariables.u[110] += acadoWorkspace.x[110];
acadoVariables.u[111] += acadoWorkspace.x[111];
acadoVariables.u[112] += acadoWorkspace.x[112];
acadoVariables.u[113] += acadoWorkspace.x[113];
acadoVariables.u[114] += acadoWorkspace.x[114];
acadoVariables.u[115] += acadoWorkspace.x[115];
acadoVariables.u[116] += acadoWorkspace.x[116];
acadoVariables.u[117] += acadoWorkspace.x[117];
acadoVariables.u[118] += acadoWorkspace.x[118];
acadoVariables.u[119] += acadoWorkspace.x[119];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
for (lRun1 = 0; lRun1 < 360; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.evGu[ 297 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.evGu[ 351 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.evGu[ 378 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.evGu[ 405 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.evGu[ 459 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.evGu[ 486 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.evGu[ 513 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.evGu[ 540 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1701 ]), &(acadoWorkspace.evGu[ 567 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1782 ]), &(acadoWorkspace.evGu[ 594 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1863 ]), &(acadoWorkspace.evGu[ 621 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.evGu[ 648 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2025 ]), &(acadoWorkspace.evGu[ 675 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2106 ]), &(acadoWorkspace.evGu[ 702 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 243 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2187 ]), &(acadoWorkspace.evGu[ 729 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.evGu[ 756 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 261 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2349 ]), &(acadoWorkspace.evGu[ 783 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.sbar[ 270 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2430 ]), &(acadoWorkspace.evGu[ 810 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.sbar[ 279 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2511 ]), &(acadoWorkspace.evGu[ 837 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.evGu[ 864 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 297 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2673 ]), &(acadoWorkspace.evGu[ 891 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.sbar[ 306 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2754 ]), &(acadoWorkspace.evGu[ 918 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.sbar[ 306 ]), &(acadoWorkspace.sbar[ 315 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2835 ]), &(acadoWorkspace.evGu[ 945 ]), &(acadoWorkspace.x[ 105 ]), &(acadoWorkspace.sbar[ 315 ]), &(acadoWorkspace.sbar[ 324 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.evGu[ 972 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.sbar[ 333 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2997 ]), &(acadoWorkspace.evGu[ 999 ]), &(acadoWorkspace.x[ 111 ]), &(acadoWorkspace.sbar[ 333 ]), &(acadoWorkspace.sbar[ 342 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3078 ]), &(acadoWorkspace.evGu[ 1026 ]), &(acadoWorkspace.x[ 114 ]), &(acadoWorkspace.sbar[ 342 ]), &(acadoWorkspace.sbar[ 351 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3159 ]), &(acadoWorkspace.evGu[ 1053 ]), &(acadoWorkspace.x[ 117 ]), &(acadoWorkspace.sbar[ 351 ]), &(acadoWorkspace.sbar[ 360 ]) );
for (lRun1 = 0; lRun1 < 369; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -7.8539816339744828e-01;
acadoVariables.lbValues[1] = -7.8539816339744828e-01;
acadoVariables.lbValues[2] = 4.9032999999999998e+00;
acadoVariables.lbValues[3] = -7.8539816339744828e-01;
acadoVariables.lbValues[4] = -7.8539816339744828e-01;
acadoVariables.lbValues[5] = 4.9032999999999998e+00;
acadoVariables.lbValues[6] = -7.8539816339744828e-01;
acadoVariables.lbValues[7] = -7.8539816339744828e-01;
acadoVariables.lbValues[8] = 4.9032999999999998e+00;
acadoVariables.lbValues[9] = -7.8539816339744828e-01;
acadoVariables.lbValues[10] = -7.8539816339744828e-01;
acadoVariables.lbValues[11] = 4.9032999999999998e+00;
acadoVariables.lbValues[12] = -7.8539816339744828e-01;
acadoVariables.lbValues[13] = -7.8539816339744828e-01;
acadoVariables.lbValues[14] = 4.9032999999999998e+00;
acadoVariables.lbValues[15] = -7.8539816339744828e-01;
acadoVariables.lbValues[16] = -7.8539816339744828e-01;
acadoVariables.lbValues[17] = 4.9032999999999998e+00;
acadoVariables.lbValues[18] = -7.8539816339744828e-01;
acadoVariables.lbValues[19] = -7.8539816339744828e-01;
acadoVariables.lbValues[20] = 4.9032999999999998e+00;
acadoVariables.lbValues[21] = -7.8539816339744828e-01;
acadoVariables.lbValues[22] = -7.8539816339744828e-01;
acadoVariables.lbValues[23] = 4.9032999999999998e+00;
acadoVariables.lbValues[24] = -7.8539816339744828e-01;
acadoVariables.lbValues[25] = -7.8539816339744828e-01;
acadoVariables.lbValues[26] = 4.9032999999999998e+00;
acadoVariables.lbValues[27] = -7.8539816339744828e-01;
acadoVariables.lbValues[28] = -7.8539816339744828e-01;
acadoVariables.lbValues[29] = 4.9032999999999998e+00;
acadoVariables.lbValues[30] = -7.8539816339744828e-01;
acadoVariables.lbValues[31] = -7.8539816339744828e-01;
acadoVariables.lbValues[32] = 4.9032999999999998e+00;
acadoVariables.lbValues[33] = -7.8539816339744828e-01;
acadoVariables.lbValues[34] = -7.8539816339744828e-01;
acadoVariables.lbValues[35] = 4.9032999999999998e+00;
acadoVariables.lbValues[36] = -7.8539816339744828e-01;
acadoVariables.lbValues[37] = -7.8539816339744828e-01;
acadoVariables.lbValues[38] = 4.9032999999999998e+00;
acadoVariables.lbValues[39] = -7.8539816339744828e-01;
acadoVariables.lbValues[40] = -7.8539816339744828e-01;
acadoVariables.lbValues[41] = 4.9032999999999998e+00;
acadoVariables.lbValues[42] = -7.8539816339744828e-01;
acadoVariables.lbValues[43] = -7.8539816339744828e-01;
acadoVariables.lbValues[44] = 4.9032999999999998e+00;
acadoVariables.lbValues[45] = -7.8539816339744828e-01;
acadoVariables.lbValues[46] = -7.8539816339744828e-01;
acadoVariables.lbValues[47] = 4.9032999999999998e+00;
acadoVariables.lbValues[48] = -7.8539816339744828e-01;
acadoVariables.lbValues[49] = -7.8539816339744828e-01;
acadoVariables.lbValues[50] = 4.9032999999999998e+00;
acadoVariables.lbValues[51] = -7.8539816339744828e-01;
acadoVariables.lbValues[52] = -7.8539816339744828e-01;
acadoVariables.lbValues[53] = 4.9032999999999998e+00;
acadoVariables.lbValues[54] = -7.8539816339744828e-01;
acadoVariables.lbValues[55] = -7.8539816339744828e-01;
acadoVariables.lbValues[56] = 4.9032999999999998e+00;
acadoVariables.lbValues[57] = -7.8539816339744828e-01;
acadoVariables.lbValues[58] = -7.8539816339744828e-01;
acadoVariables.lbValues[59] = 4.9032999999999998e+00;
acadoVariables.lbValues[60] = -7.8539816339744828e-01;
acadoVariables.lbValues[61] = -7.8539816339744828e-01;
acadoVariables.lbValues[62] = 4.9032999999999998e+00;
acadoVariables.lbValues[63] = -7.8539816339744828e-01;
acadoVariables.lbValues[64] = -7.8539816339744828e-01;
acadoVariables.lbValues[65] = 4.9032999999999998e+00;
acadoVariables.lbValues[66] = -7.8539816339744828e-01;
acadoVariables.lbValues[67] = -7.8539816339744828e-01;
acadoVariables.lbValues[68] = 4.9032999999999998e+00;
acadoVariables.lbValues[69] = -7.8539816339744828e-01;
acadoVariables.lbValues[70] = -7.8539816339744828e-01;
acadoVariables.lbValues[71] = 4.9032999999999998e+00;
acadoVariables.lbValues[72] = -7.8539816339744828e-01;
acadoVariables.lbValues[73] = -7.8539816339744828e-01;
acadoVariables.lbValues[74] = 4.9032999999999998e+00;
acadoVariables.lbValues[75] = -7.8539816339744828e-01;
acadoVariables.lbValues[76] = -7.8539816339744828e-01;
acadoVariables.lbValues[77] = 4.9032999999999998e+00;
acadoVariables.lbValues[78] = -7.8539816339744828e-01;
acadoVariables.lbValues[79] = -7.8539816339744828e-01;
acadoVariables.lbValues[80] = 4.9032999999999998e+00;
acadoVariables.lbValues[81] = -7.8539816339744828e-01;
acadoVariables.lbValues[82] = -7.8539816339744828e-01;
acadoVariables.lbValues[83] = 4.9032999999999998e+00;
acadoVariables.lbValues[84] = -7.8539816339744828e-01;
acadoVariables.lbValues[85] = -7.8539816339744828e-01;
acadoVariables.lbValues[86] = 4.9032999999999998e+00;
acadoVariables.lbValues[87] = -7.8539816339744828e-01;
acadoVariables.lbValues[88] = -7.8539816339744828e-01;
acadoVariables.lbValues[89] = 4.9032999999999998e+00;
acadoVariables.lbValues[90] = -7.8539816339744828e-01;
acadoVariables.lbValues[91] = -7.8539816339744828e-01;
acadoVariables.lbValues[92] = 4.9032999999999998e+00;
acadoVariables.lbValues[93] = -7.8539816339744828e-01;
acadoVariables.lbValues[94] = -7.8539816339744828e-01;
acadoVariables.lbValues[95] = 4.9032999999999998e+00;
acadoVariables.lbValues[96] = -7.8539816339744828e-01;
acadoVariables.lbValues[97] = -7.8539816339744828e-01;
acadoVariables.lbValues[98] = 4.9032999999999998e+00;
acadoVariables.lbValues[99] = -7.8539816339744828e-01;
acadoVariables.lbValues[100] = -7.8539816339744828e-01;
acadoVariables.lbValues[101] = 4.9032999999999998e+00;
acadoVariables.lbValues[102] = -7.8539816339744828e-01;
acadoVariables.lbValues[103] = -7.8539816339744828e-01;
acadoVariables.lbValues[104] = 4.9032999999999998e+00;
acadoVariables.lbValues[105] = -7.8539816339744828e-01;
acadoVariables.lbValues[106] = -7.8539816339744828e-01;
acadoVariables.lbValues[107] = 4.9032999999999998e+00;
acadoVariables.lbValues[108] = -7.8539816339744828e-01;
acadoVariables.lbValues[109] = -7.8539816339744828e-01;
acadoVariables.lbValues[110] = 4.9032999999999998e+00;
acadoVariables.lbValues[111] = -7.8539816339744828e-01;
acadoVariables.lbValues[112] = -7.8539816339744828e-01;
acadoVariables.lbValues[113] = 4.9032999999999998e+00;
acadoVariables.lbValues[114] = -7.8539816339744828e-01;
acadoVariables.lbValues[115] = -7.8539816339744828e-01;
acadoVariables.lbValues[116] = 4.9032999999999998e+00;
acadoVariables.lbValues[117] = -7.8539816339744828e-01;
acadoVariables.lbValues[118] = -7.8539816339744828e-01;
acadoVariables.lbValues[119] = 4.9032999999999998e+00;
acadoVariables.ubValues[0] = 7.8539816339744828e-01;
acadoVariables.ubValues[1] = 7.8539816339744828e-01;
acadoVariables.ubValues[2] = 1.4709899999999999e+01;
acadoVariables.ubValues[3] = 7.8539816339744828e-01;
acadoVariables.ubValues[4] = 7.8539816339744828e-01;
acadoVariables.ubValues[5] = 1.4709899999999999e+01;
acadoVariables.ubValues[6] = 7.8539816339744828e-01;
acadoVariables.ubValues[7] = 7.8539816339744828e-01;
acadoVariables.ubValues[8] = 1.4709899999999999e+01;
acadoVariables.ubValues[9] = 7.8539816339744828e-01;
acadoVariables.ubValues[10] = 7.8539816339744828e-01;
acadoVariables.ubValues[11] = 1.4709899999999999e+01;
acadoVariables.ubValues[12] = 7.8539816339744828e-01;
acadoVariables.ubValues[13] = 7.8539816339744828e-01;
acadoVariables.ubValues[14] = 1.4709899999999999e+01;
acadoVariables.ubValues[15] = 7.8539816339744828e-01;
acadoVariables.ubValues[16] = 7.8539816339744828e-01;
acadoVariables.ubValues[17] = 1.4709899999999999e+01;
acadoVariables.ubValues[18] = 7.8539816339744828e-01;
acadoVariables.ubValues[19] = 7.8539816339744828e-01;
acadoVariables.ubValues[20] = 1.4709899999999999e+01;
acadoVariables.ubValues[21] = 7.8539816339744828e-01;
acadoVariables.ubValues[22] = 7.8539816339744828e-01;
acadoVariables.ubValues[23] = 1.4709899999999999e+01;
acadoVariables.ubValues[24] = 7.8539816339744828e-01;
acadoVariables.ubValues[25] = 7.8539816339744828e-01;
acadoVariables.ubValues[26] = 1.4709899999999999e+01;
acadoVariables.ubValues[27] = 7.8539816339744828e-01;
acadoVariables.ubValues[28] = 7.8539816339744828e-01;
acadoVariables.ubValues[29] = 1.4709899999999999e+01;
acadoVariables.ubValues[30] = 7.8539816339744828e-01;
acadoVariables.ubValues[31] = 7.8539816339744828e-01;
acadoVariables.ubValues[32] = 1.4709899999999999e+01;
acadoVariables.ubValues[33] = 7.8539816339744828e-01;
acadoVariables.ubValues[34] = 7.8539816339744828e-01;
acadoVariables.ubValues[35] = 1.4709899999999999e+01;
acadoVariables.ubValues[36] = 7.8539816339744828e-01;
acadoVariables.ubValues[37] = 7.8539816339744828e-01;
acadoVariables.ubValues[38] = 1.4709899999999999e+01;
acadoVariables.ubValues[39] = 7.8539816339744828e-01;
acadoVariables.ubValues[40] = 7.8539816339744828e-01;
acadoVariables.ubValues[41] = 1.4709899999999999e+01;
acadoVariables.ubValues[42] = 7.8539816339744828e-01;
acadoVariables.ubValues[43] = 7.8539816339744828e-01;
acadoVariables.ubValues[44] = 1.4709899999999999e+01;
acadoVariables.ubValues[45] = 7.8539816339744828e-01;
acadoVariables.ubValues[46] = 7.8539816339744828e-01;
acadoVariables.ubValues[47] = 1.4709899999999999e+01;
acadoVariables.ubValues[48] = 7.8539816339744828e-01;
acadoVariables.ubValues[49] = 7.8539816339744828e-01;
acadoVariables.ubValues[50] = 1.4709899999999999e+01;
acadoVariables.ubValues[51] = 7.8539816339744828e-01;
acadoVariables.ubValues[52] = 7.8539816339744828e-01;
acadoVariables.ubValues[53] = 1.4709899999999999e+01;
acadoVariables.ubValues[54] = 7.8539816339744828e-01;
acadoVariables.ubValues[55] = 7.8539816339744828e-01;
acadoVariables.ubValues[56] = 1.4709899999999999e+01;
acadoVariables.ubValues[57] = 7.8539816339744828e-01;
acadoVariables.ubValues[58] = 7.8539816339744828e-01;
acadoVariables.ubValues[59] = 1.4709899999999999e+01;
acadoVariables.ubValues[60] = 7.8539816339744828e-01;
acadoVariables.ubValues[61] = 7.8539816339744828e-01;
acadoVariables.ubValues[62] = 1.4709899999999999e+01;
acadoVariables.ubValues[63] = 7.8539816339744828e-01;
acadoVariables.ubValues[64] = 7.8539816339744828e-01;
acadoVariables.ubValues[65] = 1.4709899999999999e+01;
acadoVariables.ubValues[66] = 7.8539816339744828e-01;
acadoVariables.ubValues[67] = 7.8539816339744828e-01;
acadoVariables.ubValues[68] = 1.4709899999999999e+01;
acadoVariables.ubValues[69] = 7.8539816339744828e-01;
acadoVariables.ubValues[70] = 7.8539816339744828e-01;
acadoVariables.ubValues[71] = 1.4709899999999999e+01;
acadoVariables.ubValues[72] = 7.8539816339744828e-01;
acadoVariables.ubValues[73] = 7.8539816339744828e-01;
acadoVariables.ubValues[74] = 1.4709899999999999e+01;
acadoVariables.ubValues[75] = 7.8539816339744828e-01;
acadoVariables.ubValues[76] = 7.8539816339744828e-01;
acadoVariables.ubValues[77] = 1.4709899999999999e+01;
acadoVariables.ubValues[78] = 7.8539816339744828e-01;
acadoVariables.ubValues[79] = 7.8539816339744828e-01;
acadoVariables.ubValues[80] = 1.4709899999999999e+01;
acadoVariables.ubValues[81] = 7.8539816339744828e-01;
acadoVariables.ubValues[82] = 7.8539816339744828e-01;
acadoVariables.ubValues[83] = 1.4709899999999999e+01;
acadoVariables.ubValues[84] = 7.8539816339744828e-01;
acadoVariables.ubValues[85] = 7.8539816339744828e-01;
acadoVariables.ubValues[86] = 1.4709899999999999e+01;
acadoVariables.ubValues[87] = 7.8539816339744828e-01;
acadoVariables.ubValues[88] = 7.8539816339744828e-01;
acadoVariables.ubValues[89] = 1.4709899999999999e+01;
acadoVariables.ubValues[90] = 7.8539816339744828e-01;
acadoVariables.ubValues[91] = 7.8539816339744828e-01;
acadoVariables.ubValues[92] = 1.4709899999999999e+01;
acadoVariables.ubValues[93] = 7.8539816339744828e-01;
acadoVariables.ubValues[94] = 7.8539816339744828e-01;
acadoVariables.ubValues[95] = 1.4709899999999999e+01;
acadoVariables.ubValues[96] = 7.8539816339744828e-01;
acadoVariables.ubValues[97] = 7.8539816339744828e-01;
acadoVariables.ubValues[98] = 1.4709899999999999e+01;
acadoVariables.ubValues[99] = 7.8539816339744828e-01;
acadoVariables.ubValues[100] = 7.8539816339744828e-01;
acadoVariables.ubValues[101] = 1.4709899999999999e+01;
acadoVariables.ubValues[102] = 7.8539816339744828e-01;
acadoVariables.ubValues[103] = 7.8539816339744828e-01;
acadoVariables.ubValues[104] = 1.4709899999999999e+01;
acadoVariables.ubValues[105] = 7.8539816339744828e-01;
acadoVariables.ubValues[106] = 7.8539816339744828e-01;
acadoVariables.ubValues[107] = 1.4709899999999999e+01;
acadoVariables.ubValues[108] = 7.8539816339744828e-01;
acadoVariables.ubValues[109] = 7.8539816339744828e-01;
acadoVariables.ubValues[110] = 1.4709899999999999e+01;
acadoVariables.ubValues[111] = 7.8539816339744828e-01;
acadoVariables.ubValues[112] = 7.8539816339744828e-01;
acadoVariables.ubValues[113] = 1.4709899999999999e+01;
acadoVariables.ubValues[114] = 7.8539816339744828e-01;
acadoVariables.ubValues[115] = 7.8539816339744828e-01;
acadoVariables.ubValues[116] = 1.4709899999999999e+01;
acadoVariables.ubValues[117] = 7.8539816339744828e-01;
acadoVariables.ubValues[118] = 7.8539816339744828e-01;
acadoVariables.ubValues[119] = 1.4709899999999999e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 40; ++index)
{
state[0] = acadoVariables.x[index * 9];
state[1] = acadoVariables.x[index * 9 + 1];
state[2] = acadoVariables.x[index * 9 + 2];
state[3] = acadoVariables.x[index * 9 + 3];
state[4] = acadoVariables.x[index * 9 + 4];
state[5] = acadoVariables.x[index * 9 + 5];
state[6] = acadoVariables.x[index * 9 + 6];
state[7] = acadoVariables.x[index * 9 + 7];
state[8] = acadoVariables.x[index * 9 + 8];
state[117] = acadoVariables.u[index * 3];
state[118] = acadoVariables.u[index * 3 + 1];
state[119] = acadoVariables.u[index * 3 + 2];
state[120] = acadoVariables.od[index * 9];
state[121] = acadoVariables.od[index * 9 + 1];
state[122] = acadoVariables.od[index * 9 + 2];
state[123] = acadoVariables.od[index * 9 + 3];
state[124] = acadoVariables.od[index * 9 + 4];
state[125] = acadoVariables.od[index * 9 + 5];
state[126] = acadoVariables.od[index * 9 + 6];
state[127] = acadoVariables.od[index * 9 + 7];
state[128] = acadoVariables.od[index * 9 + 8];

acado_integrate(state, index == 0);

acadoVariables.x[index * 9 + 9] = state[0];
acadoVariables.x[index * 9 + 10] = state[1];
acadoVariables.x[index * 9 + 11] = state[2];
acadoVariables.x[index * 9 + 12] = state[3];
acadoVariables.x[index * 9 + 13] = state[4];
acadoVariables.x[index * 9 + 14] = state[5];
acadoVariables.x[index * 9 + 15] = state[6];
acadoVariables.x[index * 9 + 16] = state[7];
acadoVariables.x[index * 9 + 17] = state[8];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 40; ++index)
{
acadoVariables.x[index * 9] = acadoVariables.x[index * 9 + 9];
acadoVariables.x[index * 9 + 1] = acadoVariables.x[index * 9 + 10];
acadoVariables.x[index * 9 + 2] = acadoVariables.x[index * 9 + 11];
acadoVariables.x[index * 9 + 3] = acadoVariables.x[index * 9 + 12];
acadoVariables.x[index * 9 + 4] = acadoVariables.x[index * 9 + 13];
acadoVariables.x[index * 9 + 5] = acadoVariables.x[index * 9 + 14];
acadoVariables.x[index * 9 + 6] = acadoVariables.x[index * 9 + 15];
acadoVariables.x[index * 9 + 7] = acadoVariables.x[index * 9 + 16];
acadoVariables.x[index * 9 + 8] = acadoVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[360] = xEnd[0];
acadoVariables.x[361] = xEnd[1];
acadoVariables.x[362] = xEnd[2];
acadoVariables.x[363] = xEnd[3];
acadoVariables.x[364] = xEnd[4];
acadoVariables.x[365] = xEnd[5];
acadoVariables.x[366] = xEnd[6];
acadoVariables.x[367] = xEnd[7];
acadoVariables.x[368] = xEnd[8];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[360];
state[1] = acadoVariables.x[361];
state[2] = acadoVariables.x[362];
state[3] = acadoVariables.x[363];
state[4] = acadoVariables.x[364];
state[5] = acadoVariables.x[365];
state[6] = acadoVariables.x[366];
state[7] = acadoVariables.x[367];
state[8] = acadoVariables.x[368];
if (uEnd != 0)
{
state[117] = uEnd[0];
state[118] = uEnd[1];
state[119] = uEnd[2];
}
else
{
state[117] = acadoVariables.u[117];
state[118] = acadoVariables.u[118];
state[119] = acadoVariables.u[119];
}
state[120] = acadoVariables.od[360];
state[121] = acadoVariables.od[361];
state[122] = acadoVariables.od[362];
state[123] = acadoVariables.od[363];
state[124] = acadoVariables.od[364];
state[125] = acadoVariables.od[365];
state[126] = acadoVariables.od[366];
state[127] = acadoVariables.od[367];
state[128] = acadoVariables.od[368];

acado_integrate(state, 1);

acadoVariables.x[360] = state[0];
acadoVariables.x[361] = state[1];
acadoVariables.x[362] = state[2];
acadoVariables.x[363] = state[3];
acadoVariables.x[364] = state[4];
acadoVariables.x[365] = state[5];
acadoVariables.x[366] = state[6];
acadoVariables.x[367] = state[7];
acadoVariables.x[368] = state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 39; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[117] = uEnd[0];
acadoVariables.u[118] = uEnd[1];
acadoVariables.u[119] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119];
kkt = fabs( kkt );
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 11 */
real_t tmpDy[ 11 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 11] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 11];
acadoWorkspace.Dy[lRun1 * 11 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 11 + 1];
acadoWorkspace.Dy[lRun1 * 11 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 11 + 2];
acadoWorkspace.Dy[lRun1 * 11 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 11 + 3];
acadoWorkspace.Dy[lRun1 * 11 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 11 + 4];
acadoWorkspace.Dy[lRun1 * 11 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 11 + 5];
acadoWorkspace.Dy[lRun1 * 11 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 11 + 6];
acadoWorkspace.Dy[lRun1 * 11 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 11 + 7];
acadoWorkspace.Dy[lRun1 * 11 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 11 + 8];
acadoWorkspace.Dy[lRun1 * 11 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 11 + 9];
acadoWorkspace.Dy[lRun1 * 11 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 11 + 10];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[360];
acadoWorkspace.objValueIn[1] = acadoVariables.x[361];
acadoWorkspace.objValueIn[2] = acadoVariables.x[362];
acadoWorkspace.objValueIn[3] = acadoVariables.x[363];
acadoWorkspace.objValueIn[4] = acadoVariables.x[364];
acadoWorkspace.objValueIn[5] = acadoVariables.x[365];
acadoWorkspace.objValueIn[6] = acadoVariables.x[366];
acadoWorkspace.objValueIn[7] = acadoVariables.x[367];
acadoWorkspace.objValueIn[8] = acadoVariables.x[368];
acadoWorkspace.objValueIn[9] = acadoVariables.od[360];
acadoWorkspace.objValueIn[10] = acadoVariables.od[361];
acadoWorkspace.objValueIn[11] = acadoVariables.od[362];
acadoWorkspace.objValueIn[12] = acadoVariables.od[363];
acadoWorkspace.objValueIn[13] = acadoVariables.od[364];
acadoWorkspace.objValueIn[14] = acadoVariables.od[365];
acadoWorkspace.objValueIn[15] = acadoVariables.od[366];
acadoWorkspace.objValueIn[16] = acadoVariables.od[367];
acadoWorkspace.objValueIn[17] = acadoVariables.od[368];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 11]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 11 + 1]*acadoVariables.W[12];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 11 + 2]*acadoVariables.W[24];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 11 + 3]*acadoVariables.W[36];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 11 + 4]*acadoVariables.W[48];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 11 + 5]*acadoVariables.W[60];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 11 + 6]*acadoVariables.W[72];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 11 + 7]*acadoVariables.W[84];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 11 + 8]*acadoVariables.W[96];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 11 + 9]*acadoVariables.W[108];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 11 + 10]*acadoVariables.W[120];
objVal += + acadoWorkspace.Dy[lRun1 * 11]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 11 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 11 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 11 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 11 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 11 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 11 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 11 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 11 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 11 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 11 + 10]*tmpDy[10];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[7];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[14];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[21];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[28];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[35];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

