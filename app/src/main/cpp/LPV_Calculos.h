//
// Created by Asus on 22/09/2021.
//

#ifndef QUADROTORCONTROLUV6_LPV_CALCULOS_H
#define QUADROTORCONTROLUV6_LPV_CALCULOS_H

#endif //QUADROTORCONTROLUV6_LPV_CALCULOS_H

#include <iostream>
#include <jni.h>
#include <string>
#include <cstdlib>
#include "Eigen/Dense"
//#include "Eigen"
#include <math.h>
#include <bitset>

#include <android/log.h>

#include "MatricesLPV_AltHold.h"

void Matrices_Stabilice();
void Matrices_Loiter();
void alpha_eval(double par[]);
void alpha_eval_AltHold(double par[]);
void actualizar_controlador_stabilize(double error[]);
void actualizar_controlador_AltHold(double error[]);
void parameter_stabilize(double dPhi, double dPsi);
void parameter_AltHold(double theta, double phi, double d_phi, double d_psi);

void cambio_de_variable(double refPsi, double Vx, double Vy, double Vz);

using namespace Eigen;
using namespace std;
using std::bitset;

MatrixXf Ak_stab(12,12);
MatrixXf Bk_stab(12,3);
MatrixXf Ck_stab(3,12);

MatrixXf Ak1_stab(12,12), Ak2_stab(12,12), Ak3_stab(12,12), Ak4_stab(12,12);
MatrixXf Bk1_stab(12,3), Bk2_stab(12,3), Bk3_stab(12,3), Bk4_stab(12,3);
MatrixXf Ck1_stab(3,12), Ck2_stab(3,12), Ck3_stab(3,12), Ck4_stab(3,12);
MatrixXf K_lqi(3,9);

VectorXf alpha_stab(4), alpha_AltHold(8);
VectorXf dXc_stab(12), dXc_AltHold(16);
VectorXf Xc_stab(12), Xc_AltHold(16);
VectorXf error_stab(3), error_AltHold(4), I_error_Loiter(3),error_Loiter_au(9);
VectorXf U_stab(3), U_AltHold(4);
VectorXf Vxyz(3);

double prueba_par_stab[2]={3,4};
double par_stab[2], par_AltHold[3], alpha_AltHold_aux[8]; //Parámetros para controladores
double pi = 3.141516;
double Pitch_L, Roll_L, U1_L;

void asignar_matrices_controlador(){
    Matrices_Stabilice();
    Matrices_AltHold();
    Matrices_Loiter();
}

void calculando_controlador_stabilize(double dPhi, double dPsi, double error[]){
    parameter_stabilize(dPhi, dPsi);
    alpha_eval(par_stab);
    actualizar_controlador_stabilize(error);
    //__android_log_print(ANDROID_LOG_ERROR, "LPV Calculos "," a1%f a2%f a3%f a4%f",alpha_stab[0],alpha_stab[1],alpha_stab[2],alpha_stab[3]);
}

void calculando_controlador_AltHold(double theta, double phi, double d_phi, double d_psi, double error[]){
    parameter_AltHold(theta,phi,d_phi,d_psi);
    alpha_eval_AltHold(par_AltHold);
    actualizar_controlador_AltHold(error);
}

void calculando_controlador_Loiter(double Psi, double Theta, double Phi, double dPhi, double dPsi,double refPsi, double error_Loiter_Est[], double error_Loiter[]){
    error_Loiter_au <<error_Loiter_Est[0],error_Loiter_Est[1],error_Loiter_Est[2],error_Loiter_Est[3],error_Loiter_Est[4],error_Loiter_Est[5],I_error_Loiter[0],I_error_Loiter[1],I_error_Loiter[2];
    I_error_Loiter << error_Loiter[0],error_Loiter[1],error_Loiter[2];
    Vxyz = K_lqi*error_Loiter_au;
    cambio_de_variable(refPsi,Vxyz[0],Vxyz[1],Vxyz[2]);
    double error_post[] = {refPsi-Psi, Pitch_L-Theta, Roll_L-Phi};
    calculando_controlador_stabilize(dPhi,dPsi,error_post);
    __android_log_print(ANDROID_LOG_ERROR, "LPV Calculos "," Pitch_L%f Roll_L%f U1_L%f ",Pitch_L,Roll_L,U1_L);
    //__android_log_print(ANDROID_LOG_ERROR, "LPV Calculos "," Vx%f Vy%f Vz%f ",Vxyz[0],Vxyz[1],Vxyz[2]);
}

void cambio_de_variable(double refPsi, double Vx, double Vy, double Vz){
    double g = 9.7967, m = 1.285;
    double a = Vx/(Vz + g);
    double b = Vy/(Vz + g);
    double c = cos(refPsi);
    double d = sin(refPsi);
    Pitch_L = atan(c*a + d*b);
    double epsi = pi/4;
    if ((abs(refPsi) < epsi) || (abs(refPsi) < pi - epsi)){
        Roll_L = atan((sin(Pitch_L)*d - cos(Pitch_L)*b)/c);
    } else{
        Roll_L = atan((cos(Pitch_L) - sin(Pitch_L)*c)/d);
    }
    U1_L = m*(Vz + g)/(cos(Roll_L)*cos(Pitch_L));
}


void parameter_stabilize(double dPhi, double dPsi){
    par_stab[0] = dPhi;
    par_stab[1] = dPsi;
}

void parameter_AltHold(double theta, double phi, double d_phi, double d_psi){
    par_AltHold[0] = cos(theta)*cos(phi);
    par_AltHold[1] = d_phi;
    par_AltHold[2] = d_psi;
}


void alpha_eval(double par[]){
    double the1b = -180*pi/180; double the1a = 180*pi/180;
    double the2b = -90*pi/180; double the2a = 90*pi/180;


    double dthe1 = the1a - the1b; double dthe2 = the2a - the2b;

    double alpha1 = ((the1a - par[1])/dthe1)*((the2a - par[2])/dthe2);
    double alpha2 = ((the1a - par[1])/dthe1)*((par[2] - the2b)/dthe2);
    double alpha3 = ((par[1] - the1b)/dthe1)*((the2a - par[2])/dthe2);
    double alpha4 = ((par[1] - the1b)/dthe1)*((par[2] - the2b)/dthe2);

    alpha_stab << alpha1,alpha2,alpha3,alpha4;
    //__android_log_print(ANDROID_LOG_ERROR, "TRACKERS desde LPV controller", "%s","Alpha Eval fin");
}


void alpha_eval_AltHold(double par[]){
    double the1min = -1;    double the2min = -180*pi/180;     double the3min = -90*pi/180;
    double the1max = 1;     double the2max = 180*pi/180;      double the3max = 90*pi/180;

    double dthe1 = the1max - the1min; double dthe2 = the2max - the2min; double  dthe3 = the3max - the3min;

    double par1b = (the1max - par[1])/dthe1;
    double par1a = (par[1] - the1min)/dthe1;

    double par2b = (the2max - par[2])/dthe2;
    double par2a = (par[2] - the2min)/dthe2;

    double par3b = (the3max - par[3])/dthe3;
    double par3a = (par[3] - the3min)/dthe3;

    double alpar1,alpar2,alpar3;

    for (int i = 8; i < 16; ++i) {
        bitset<4> ap(13);
        if (ap.test(3)==0){
            alpar1 = par1b;
        } else{
            alpar1 = par1a;
        }
        if (ap.test(2)==0){
            alpar2 = par2b;
        } else{
            alpar2 = par2a;
        }
        if (ap.test(1)==0){
            alpar3 = par3b;
        } else{
            alpar3 = par3a;
        }


        alpha_AltHold_aux[i-8] = alpar1 * alpar2 * alpar3;
    }
    alpha_AltHold << alpha_AltHold_aux[0],alpha_AltHold_aux[1],alpha_AltHold_aux[2],alpha_AltHold_aux[3],alpha_AltHold_aux[4],alpha_AltHold_aux[5],alpha_AltHold_aux[6],alpha_AltHold_aux[7];
    __android_log_print(ANDROID_LOG_ERROR, "TRACKERS desde LPV controller", "%s","Alpha Eval Altitud Hold fin");
    //__android_log_print(ANDROID_LOG_ERROR, "TRACKERS desde LPV controller", "%u %u %u %u",bs3.test(0),bs3.test(1),bs3.test(2),bs3.test(3));
}


void actualizar_controlador_stabilize(double error[]){
        error_stab <<error[0],error[1],error[2];
        Ak_stab = Ak_stab + alpha_stab[0]*Ak1_stab + alpha_stab[1]*Ak2_stab + alpha_stab[2]*Ak3_stab + alpha_stab[3]*Ak4_stab;
        Bk_stab = Bk_stab + alpha_stab[0]*Bk1_stab + alpha_stab[1]*Bk2_stab + alpha_stab[2]*Bk3_stab + alpha_stab[3]*Bk4_stab;
        Ck_stab = Ck_stab + alpha_stab[0]*Ck1_stab + alpha_stab[1]*Ck2_stab + alpha_stab[2]*Ck3_stab + alpha_stab[3]*Ck4_stab;

        dXc_stab = Ak_stab*Xc_stab + Bk_stab*error_stab;
        U_stab = Ck_stab*Xc_stab;

        Xc_stab = dXc_stab;  //Derivada discreta Xc(n) = Xc(n-1)
    //__android_log_print(ANDROID_LOG_ERROR, "Calculando controlador", "a %s ","Matrices actualizadas");
}

void actualizar_controlador_AltHold(double error[]){
    error_AltHold <<error[0],error[1],error[2],error[3];
    Ak_AltHold = Ak_AltHold + alpha_AltHold[0]*Ak1_AltHold + alpha_AltHold[1]*Ak2_AltHold + alpha_AltHold[2]*Ak3_AltHold + alpha_AltHold[3]*Ak4_AltHold + alpha_AltHold[4]*Ak5_AltHold + alpha_AltHold[5]*Ak6_AltHold + alpha_AltHold[6]*Ak7_AltHold + alpha_AltHold[7]*Ak8_AltHold;
    Bk_AltHold = Bk_AltHold + alpha_AltHold[0]*Bk1_AltHold + alpha_AltHold[1]*Bk2_AltHold + alpha_AltHold[2]*Bk3_AltHold + alpha_AltHold[3]*Bk4_AltHold + alpha_AltHold[4]*Bk5_AltHold + alpha_AltHold[5]*Bk6_AltHold + alpha_AltHold[6]*Bk7_AltHold + alpha_AltHold[7]*Bk8_AltHold;
    Ck_AltHold = Ck_AltHold + alpha_AltHold[0]*Ck1_AltHold + alpha_AltHold[1]*Ck2_AltHold + alpha_AltHold[2]*Ck3_AltHold + alpha_AltHold[3]*Ck4_AltHold + alpha_AltHold[4]*Ck5_AltHold + alpha_AltHold[5]*Ck6_AltHold + alpha_AltHold[6]*Ck7_AltHold + alpha_AltHold[7]*Ck8_AltHold;

    dXc_AltHold = Ak_AltHold*Xc_AltHold + Bk_AltHold*error_AltHold;
    U_AltHold = Ck_AltHold*Xc_AltHold;

    Xc_AltHold = dXc_AltHold;
    __android_log_print(ANDROID_LOG_ERROR, "Calculando controlador", "a %s ","Matrices AltHold actualizadas");
}


VectorXf leer_U_stabilize(){

    return U_stab;
}

VectorXf leer_U_AltHold(){
    return U_AltHold;
}

VectorXf leer_U_Loiter(){
    VectorXf U_Loiter(4);
    VectorXf U_stabilize(3);
    U_stabilize<< leer_U_stabilize();
    U_Loiter << U1_L,U_stabilize[0],U_stabilize[1],U_stabilize[2];

    return U_Loiter;
}

void Matrices_Loiter(){
    K_lqi << 43.1662,  13.6504,   0.0000,  -0.0000,   0.0000,   0.0000,  31.6228,  -0.0000,  -0.0000,
              0.0000,   0.0000,  43.1662,  13.6504,  -0.0000,   0.0000,   0.0000,  31.6228,   0.0000,
             -0.0000,   0.0000,   0.0000,   0.0000,  43.1662,  13.6504,  -0.0000,   0.0000,  31.6228;
}

void Matrices_Stabilice(){
   Ak1_stab << -0.0005,  -0.0003,   0.0085,  -0.0001,   0.0003,  -0.0002,   0.0000,  -0.0000,  -0.0000,   0.0001,  -0.0001,  0.0000,
               -0.0000,  -0.0016,   0.0001,   0.0000,  -0.0013,   0.0001,  -0.0004,  -0.0001,   0.0000,  -0.0080,   0.3103, -0.9305,
                0.0001,   0.0001,  -0.0026,   0.0000,  -0.0000,   0.0001,   0.0000,   0.0010,  -0.0001,   0.0949,  -0.0778,  0.0282,
                0.0000,   0.0001,   0.0000,  -0.0038,  -0.0002,  -0.0016,   0.0000,   0.0001,   0.0024,   0.0059,   1.0167,  0.5512,
                0.0000,  -0.0012,  -0.0001,  -0.0003,  -0.0014,  -0.0001,  -0.0005,   0.0000,   0.0003,   0.0012,   0.4644, -1.0684,
               -0.0000,   0.0002,   0.0002,  -0.0017,  -0.0000,  -0.0009,   0.0001,  -0.0000,   0.0015,  -0.0013,   0.6269,  0.4201,
               -0.0000,  -0.0061,  -0.0004,   0.0003,  -0.0061,   0.0005,  -0.0023,  -0.0000,  -0.0000,  -0.0014,   1.5465, -5.0227,
               -0.0002,  -0.0003,   0.0051,   0.0001,   0.0001,  -0.0002,  -0.0000,  -0.0019,   0.0002,  -0.1820,   0.1299, -0.0835,
                0.0000,  -0.0001,  -0.0003,   0.0048,   0.0002,   0.0021,  -0.0000,   0.0000,  -0.0034,   0.0011,  -1.4367, -0.7865,
               -0.0000,  -0.0000,   0.0004,   0.0000,   0.0000,  -0.0000,  -0.0000,  -0.0002,   0.0000,  -0.0150,   0.0109, -0.0027,
                0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0121, -0.0002,
                0.0000,  -0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,  -0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0003, -0.0180;
   Ak1_stab << Ak1_stab*100000;

   Ak2_stab << -0.0005,  -0.0003,   0.0085,  -0.0001,   0.0003,  -0.0002,   0.0000,  -0.0000,   0.0000,   0.0001,  -0.0001,  0.0002,
               -0.0000,  -0.0016,   0.0002,   0.0000,  -0.0012,   0.0001,  -0.0004,  -0.0001,   0.0000,  -0.0103,   0.2942, -0.9267,
                0.0001,   0.0004,  -0.0024,  -0.0001,   0.0002,   0.0000,   0.0001,   0.0010,  -0.0000,   0.0901,  -0.0966,  0.2122,
                0.0000,   0.0001,   0.0000,  -0.0034,  -0.0002,  -0.0014,   0.0000,   0.0001,   0.0022,   0.0051,   0.9207,  0.5250,
               -0.0000,  -0.0012,   0.0000,  -0.0003,  -0.0013,  -0.0001,  -0.0005,  -0.0000,   0.0002,  -0.0022,   0.4336, -1.0464,
               -0.0000,   0.0001,   0.0001,  -0.0015,   0.0000,  -0.0008,   0.0000,  -0.0000,   0.0014,  -0.0014,   0.5683,  0.3922,
               -0.0000,  -0.0060,   0.0001,   0.0004,  -0.0060,   0.0005,  -0.0022,  -0.0002,  -0.0001,  -0.0158,   1.4595, -4.9191,
               -0.0002,  -0.0009,   0.0048,   0.0003,  -0.0003,  -0.0000,  -0.0002,  -0.0018,   0.0000,  -0.1728,   0.1678, -0.4351,
                0.0000,  -0.0001,  -0.0003,   0.0043,   0.0002,   0.0019,  -0.0000,   0.0000,  -0.0030,   0.0017,  -1.3034, -0.7311,
               -0.0000,  -0.0001,   0.0003,   0.0000,  -0.0000,  -0.0000,  -0.0000,  -0.0002,   0.0000,  -0.0142,   0.0140, -0.0318,
                0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0110,  0.0001,
               -0.0000,  -0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,  -0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0005, -0.0175;
   Ak2_stab << Ak2_stab*100000;

   Ak3_stab << -0.0005,  -0.0003,   0.0085,  -0.0001,   0.0003,  -0.0002,   0.0000,  -0.0000,   0.0000,   0.0001,  -0.0000, -0.0000,
               -0.0000,  -0.0016,   0.0001,   0.0000,  -0.0013,   0.0001,  -0.0004,  -0.0001,   0.0000,  -0.0080,   0.3068, -0.9327,
                0.0001,   0.0001,  -0.0026,  -0.0001,  -0.0001,   0.0001,  -0.0000,   0.0010,  -0.0001,   0.0964,  -0.0374, -0.0066,
                0.0000,   0.0001,   0.0000,  -0.0038,  -0.0002,  -0.0016,   0.0000,   0.0001,   0.0024,   0.0055,   1.0233,  0.5496,
                0.0000,  -0.0013,  -0.0001,  -0.0003,  -0.0014,  -0.0001,  -0.0005,   0.0000,   0.0003,   0.0014,   0.4651, -1.0751,
               -0.0000,   0.0002,   0.0002,  -0.0017,  -0.0000,  -0.0009,   0.0001,  -0.0000,   0.0015,  -0.0016,   0.6290,  0.4214,
               -0.0000,  -0.0062,  -0.0004,   0.0003,  -0.0062,   0.0005,  -0.0023,  -0.0000,  -0.0000,  -0.0005,   1.5447, -5.0501,
               -0.0002,  -0.0002,   0.0052,   0.0002,   0.0002,  -0.0001,  -0.0000,  -0.0020,   0.0001,  -0.1847,   0.0523, -0.0170,
                0.0000,  -0.0001,  -0.0003,   0.0048,   0.0002,   0.0021,  -0.0000,   0.0000,  -0.0034,   0.0018,  -1.4421, -0.7878,
               -0.0000,  -0.0000,   0.0004,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0002,   0.0000,  -0.0152,   0.0045,  0.0028,
                0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0121, -0.0002,
                0.0000,  -0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0000,   0.0003, -0.0181;
   Ak3_stab << Ak3_stab*100000;

   Ak4_stab << -0.0005,  -0.0003,   0.0085,  -0.0001,   0.0003,  -0.0002,   0.0000,  -0.0000,  -0.0000,   0.0001,  -0.0001, -0.0000,
               -0.0000,  -0.0016,   0.0001,   0.0001,  -0.0013,   0.0001,  -0.0004,  -0.0001,  -0.0000,  -0.0080,   0.2889, -0.9636,
                0.0001,   0.0001,  -0.0027,  -0.0001,  -0.0001,   0.0001,   0.0000,   0.0010,  -0.0001,   0.0979,  -0.0583,  0.0163,
                0.0000,   0.0001,   0.0000,  -0.0033,  -0.0002,  -0.0014,   0.0000,   0.0001,   0.0021,   0.0059,   0.8986,  0.4962,
                0.0000,  -0.0013,  -0.0001,  -0.0002,  -0.0014,  -0.0000,  -0.0005,   0.0000,   0.0002,   0.0016,   0.4293, -1.1150,
               -0.0000,   0.0002,   0.0001,  -0.0015,   0.0000,  -0.0007,   0.0001,  -0.0000,   0.0013,  -0.0015,   0.5524,  0.3884,
                0.0000,  -0.0063,  -0.0004,   0.0007,  -0.0063,   0.0007,  -0.0023,  -0.0000,  -0.0003,   0.0001,   1.4500, -5.2010,
               -0.0002,  -0.0003,   0.0053,   0.0002,   0.0001,  -0.0001,  -0.0000,  -0.0020,   0.0001,  -0.1877,   0.0951, -0.0600,
                0.0000,  -0.0001,  -0.0003,   0.0042,   0.0002,   0.0019,  -0.0000,   0.0000,  -0.0030,   0.0015,  -1.2686, -0.7107,
               -0.0000,  -0.0000,   0.0004,   0.0000,   0.0000,  -0.0000,  -0.0000,  -0.0002,   0.0000,  -0.0155,   0.0079, -0.0007,
                0.0000,   0.0000,  -0.0000,   0.0000,   0.0000,   0.0000,   0.0000,   0.0000,  -0.0000,   0.0000,  -0.0108,  0.0005,
                0.0000,  -0.0000,  -0.0000,   0.0000,  -0.0000,   0.0000,  -0.0000,  -0.0000,  -0.0000,   0.0000,   0.0006, -0.0183;
   Ak4_stab << Ak4_stab*100000;

   Bk1_stab << -0.0000,    0.0002,    0.0047,
               -0.0096,   -0.3471,    0.0001,
                0.8664,   -0.0204,   -0.0000,
                0.0157,    0.3317,    0.0188,
                0.1577,   -6.1206,    0.0019,
               -0.0482,    0.4726,    0.0422,
              -10.1323,  561.3721,   -0.1951,
              522.1965,    5.5603,    0.2283,
               -1.9200,    6.1925,  160.2270,
              -33.8653,   -0.0709,   -0.0901,
                0.0177,    0.5038,   -1.5304,
                0.0352,   -1.7204,   -0.6211;

   Bk2_stab << -0.0000,    0.0002,    0.0047,
               -0.0096,   -0.3471,    0.0006,
                0.8664,   -0.0204,    0.0001,
                0.0159,    0.3316,    0.0198,
                0.1576,   -6.1207,    0.0026,
               -0.0481,    0.4725,    0.0428,
              -10.1325,  561.3718,   -0.1929,
              522.1964,    5.5604,    0.2281,
               -1.9203,    6.1926,  160.2256,
              -33.8653,   -0.0709,   -0.0901,
                0.0177,    0.5038,   -1.5304,
                0.0352,   -1.7204,   -0.6211;

   Bk3_stab << -0.0000,    0.0002,    0.0047,
               -0.0096,   -0.3471,    0.0004,
                0.8663,   -0.0204,   -0.0002,
                0.0157,    0.3305,    0.0235,
                0.1577,   -6.1207,    0.0027,
               -0.0483,    0.4718,    0.0451,
              -10.1323,  561.3721,   -0.1942,
              522.1965,    5.5604,    0.2285,
               -1.9200,    6.1941,  160.2204,
              -33.8653,   -0.0709,   -0.0901,
                0.0177,    0.5038,   -1.5304,
                0.0352,   -1.7204,   -0.6211;

   Bk4_stab << -0.0000,    0.0002,    0.0047,
               -0.0095,   -0.3470,    0.0005,
                0.8664,   -0.0204,   -0.0001,
                0.0158,    0.3320,    0.0221,
                0.1578,   -6.1205,    0.0027,
               -0.0482,    0.4728,    0.0443,
              -10.1319,  561.3722,   -0.1936,
              522.1965,    5.5604,    0.2283,
               -1.9202,    6.1920,  160.2223,
              -33.8653,   -0.0709,   -0.0901,
                0.0177,    0.5038,   -1.5304,
                0.0352,   -1.7204,   -0.6212;

    Ck1_stab << 0.4458,    0.3114,   -7.5568,    0.1072,   -0.2668,    0.1383,   -0.0033,    0.0118,    0.0000,   -0.0722,    0.0754,   -0.0085,
                0.0056,   10.4170,    0.7336,   -0.3955,    2.5722,   -0.2017,    0.0344,   -0.0011,    0.0001,   -0.0013,    0.3896,   -1.2902,
                0.0063,   -0.4623,    0.3371,   -8.9036,   -0.6157,   -2.0688,    0.0000,   -0.0003,   -0.0008,   -0.0015,   -1.0813,   -0.4683;

    Ck2_stab << 0.4458,    0.3112,   -7.5569,    0.1073,   -0.2669,    0.1383,   -0.0034,    0.0118,   -0.0000,   -0.0686,    0.0881,   -0.1492,
                0.0056,   10.4170,    0.7338,   -0.3955,    2.5723,   -0.2017,    0.0344,   -0.0012,    0.0000,   -0.0049,    0.3682,   -1.2651,
                0.0063,   -0.4623,    0.3371,   -8.9040,   -0.6158,   -2.0689,    0.0000,   -0.0003,   -0.0005,   -0.0006,   -0.9815,   -0.4345;

    Ck3_stab << 0.4458,    0.3114,   -7.5568,    0.1073,   -0.2667,    0.1383,   -0.0033,    0.0118,   -0.0000,   -0.0733,    0.0447,    0.0180,
                0.0056,   10.4170,    0.7336,   -0.3955,    2.5722,   -0.2017,    0.0344,   -0.0011,    0.0001,   -0.0010,    0.3888,   -1.2969,
                0.0063,   -0.4623,    0.3371,   -8.9036,   -0.6157,   -2.0688,    0.0000,   -0.0003,   -0.0008,   -0.0010,   -1.0861,   -0.4678;

    Ck4_stab << 0.4458,    0.3114,   -7.5567,    0.1073,   -0.2667,    0.1383,   -0.0033,    0.0118,    0.0000,   -0.0745,    0.0586,   -0.0002,
                0.0056,   10.4170,    0.7336,   -0.3954,    2.5722,   -0.2017,    0.0343,   -0.0011,   -0.0000,   -0.0009,    0.3656,   -1.3354,
                0.0063,   -0.4623,    0.3372,   -8.9041,   -0.6158,   -2.0689,    0.0000,   -0.0003,   -0.0005,   -0.0013,   -0.9568,   -0.4087;

    __android_log_print(ANDROID_LOG_ERROR, "TRACKERS desde LPV controller", "%s","Matrices completadas");
}
