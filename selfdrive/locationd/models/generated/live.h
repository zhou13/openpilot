#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7839301313217597888);
void live_err_fun(double *nom_x, double *delta_x, double *out_1267936703929999891);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7099514616203232870);
void live_H_mod_fun(double *state, double *out_7330774590226362058);
void live_f_fun(double *state, double dt, double *out_3398589702823944963);
void live_F_fun(double *state, double dt, double *out_682249849216058718);
void live_h_3(double *state, double *unused, double *out_6521397066358485629);
void live_H_3(double *state, double *unused, double *out_6408710421213866743);
void live_h_4(double *state, double *unused, double *out_2613013462545532889);
void live_H_4(double *state, double *unused, double *out_3709093966306788222);
void live_h_9(double *state, double *unused, double *out_4817299262344375093);
void live_H_9(double *state, double *unused, double *out_7537673646166706998);
void live_h_10(double *state, double *unused, double *out_5321869332625600953);
void live_H_10(double *state, double *unused, double *out_5159250767268761236);
void live_h_12(double *state, double *unused, double *out_3714629976958486470);
void live_H_12(double *state, double *unused, double *out_717902807734686747);
void live_h_31(double *state, double *unused, double *out_8886542565457460456);
void live_H_31(double *state, double *unused, double *out_4863412381150342658);
void live_h_32(double *state, double *unused, double *out_1795446354464660300);
void live_H_32(double *state, double *unused, double *out_5503135170868419303);
void live_h_13(double *state, double *unused, double *out_5771935272236362696);
void live_H_13(double *state, double *unused, double *out_3522645509573237753);
void live_h_14(double *state, double *unused, double *out_4817299262344375093);
void live_H_14(double *state, double *unused, double *out_7537673646166706998);
void live_h_19(double *state, double *unused, double *out_6165822868946389536);
void live_H_19(double *state, double *unused, double *out_4620872070863921362);
void live_h_33(double *state, double *unused, double *out_7285358876662887127);
void live_H_33(double *state, double *unused, double *out_4966881605278445816);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}