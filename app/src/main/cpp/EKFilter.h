//
// Created by Pedro - Juan on 26/09/2018 UNIVERSIDAD DEL VALLE.
//

#ifndef QUADROTORCONTROLUV_APP_EKFILTER_H
#define QUADROTORCONTROLUV_APP_EKFILTER_H
#endif //QUADROTORCONTROLUV_APP_EKFILTER_H

#include <iostream>
#include <jni.h>
#include <string>
#include <cstdlib>
#include "Eigen/Dense"
//#include "Eigen"
#include <math.h>

#include <android/log.h>

void matrix_A_k(float U1);
void matrixQ_kf();
void matrixR_kf();
void matrixH();

using namespace Eigen;

float varQ  = 0.001;    // Variable matriz Q
float varR = 1;
float T     = 0.01;     // Tiempo de muestreo
float m     = 1.568;    // Masa
float g     = 9.807;    // Gravedad
float Ixx   = 0.0135;   // Inercia en x
float Iyy   = 0.0124;   // Inercia en y
float Izz   = 0.0336;   // Inercia en z

// CREACION DE VECTORES Y MATRICES

MatrixXf A_k(12, 12);    // Matriz A 12x12
MatrixXf A_kt(12, 12);   // Matriz A transpuesta
MatrixXf H(9, 12);       // Matriz H 4x12
MatrixXf Ht(12, 9);      // Matriz H transpuesta
MatrixXf Q_kf(12, 12);   // Matriz Q 12x12
MatrixXf R_kf(9, 9);     // Matriz R 4x4
MatrixXf inv_(9,9);      // Matriz de 4x4 usada en la ecuacion de
MatrixXf w(12, 12);      // Matriz identidad w que sera usada en la ecuacion de actualizacion EKF
MatrixXf Z_k(9,1);       // Creacion de matriz de sensores de 4x1 (Z_k)
VectorXf Xk_(12);        // Creacion de un vector estados de 12 (Xk_) SetZero
VectorXf X_k(12);
VectorXf X_k_1(12);      // Creacion de un vector de 12 (X_k_1)
MatrixXf Pk_(12,12);     // Creacion de una matriz de 12x12 (P_k_)
MatrixXf P_k(12,12);
MatrixXf P_k_1(12,12);   // Creacion de una matriz de 12x12 (P_k_1)
MatrixXf K_k(12, 9);

VectorXf inter(12);

void initPositionEKF(double ic[], double Q_val, double R_val, double m_, double Ixx_, double Iyy_, double Izz_) {
    Ixx= Ixx_;
    Iyy= Iyy_;
    Izz= Izz_;
    m= m_;

    X_k_1 << ic[0],ic[1],ic[2],ic[3],ic[4],ic[5],ic[6],ic[7],ic[8],ic[9],ic[10],ic[11];
    //Xk_ << ic[0],ic[1],ic[2],ic[3],ic[4],ic[5],ic[6],ic[7],ic[8],ic[9],ic[10],ic[11]; deshabilite ESTA
    //X_k_1.setZero(12);                    // Llenando vector de 12 con "Ceros"
    Xk_.setZero(12);                      // Llenando vector de 12 con "Ceros" HABILITE ESTA
    P_k_1 =100*P_k_1.setIdentity(12,12);  // Matriz identidad
    w.setIdentity(12,12);                 // Matriz identidad
    Pk_.setZero(12,12);

    varQ = Q_val;
    varR = R_val;
    matrixQ_kf();
    matrixR_kf();
    matrixH();

    __android_log_print(ANDROID_LOG_ERROR, "TRACKERS desde EKFilter", "%f",X_k_1(11));
}

//float matrizw(){
// w.setIdenty(12,12);
//}

void matrix_A_k(float U1){

    A_k <<  1, T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, ((T*U1/m)*(cos(X_k_1(6))*sin(X_k_1(10))-sin(X_k_1(6))*sin(X_k_1(8))*cos(X_k_1(10)))), 0, ((T*U1/m)*(cos(X_k_1(6))*cos(X_k_1(8)*cos(X_k_1(10))))), 0, ((T*U1/m)*(sin(X_k_1(6))*cos(X_k_1(10))+cos(X_k_1(6))*sin(X_k_1(8))*(-sin(X_k_1(10))))), 0,
            0, 0, 1, T, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, ((T*U1/m)*(-cos(X_k_1(6))*sin(X_k_1(8))*cos(X_k_1(10))-sin(X_k_1(6))*sin(X_k_1(10)))), 0, ((T*U1/m)*(-sin(X_k_1(6))*cos(X_k_1(8))*cos(X_k_1(10)))), 0, ((T*U1/m)*(-sin(X_k_1(6))*sin(X_k_1(8))*(-sin(X_k_1(10)))+cos(X_k_1(6))*cos(X_k_1(10)))), 0,
            0, 0, 0, 0, 1, T, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, ((T*U1/m)*(-sin(X_k_1(8))*cos(X_k_1(10)))), 0, ((T*U1/m)*(-sin(X_k_1(10))*cos(X_k_1(8)))), 0,
            0, 0, 0, 0, 0, 0, 1, T, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, ((T*(X_k_1(11))*((Ixx-Iyy)/Izz))), 0, ((T*(X_k_1(9))*((Ixx-Iyy)/Izz))),
            0, 0, 0, 0, 0, 0, 0, 0, 1, T, 0, 0,
            0, 0, 0, 0, 0, 0, 0, ((T*(X_k_1(11))*((Izz-Ixx)/Iyy))), 0, 1, 0, ((T*(X_k_1(7))*((Izz-Ixx)/Iyy))),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, T,
            0, 0, 0, 0, 0, 0, 0, ((T*(X_k_1(9))*((Iyy-Izz)/Ixx))), 0, ((T*(X_k_1(7))*((Iyy-Izz)/Ixx))), 0, 1;

    A_kt << A_k.transpose();

}
/*
void matrix_B_k(){
    MatrixXf B_k(12, 4);  // Matriz B 12x4

    B_k <<  0, 0, 0, 0,
            ((T/m)*(sin(X_k_1(6))*sin(X_k_1(10))+cos(X_k_1(6))*sin(X_k_1(8))*cos(X_k_1(10)))), 0, 0, 0,
            0, 0, 0, 0,
            ((T/m)*(-sin(X_k_1(6))*sin(X_k_1(8))*cos(X_k_1(10))+cos(X_k_1(6))*sin(X_k_1(10)))), 0, 0, 0,
            0, 0, 0, 0,
            ((T/m)*((cos(X_k_1(8))*cos(X_k_1(10))))), 0, 0, 0,
            0, 0, 0, 0,
            0, (T/Izz), 0, 0,
            0, 0, 0, 0,
            0, 0, (T/Iyy), 0,
            0, 0, 0, 0,
            0, 0, 0, (T/Ixx);
}
*/
void matrixH() {


    H   <<  1, T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, T, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, T, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, T, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, T, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, T, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, T, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, T, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, T;

    Ht << H.transpose();
}

void matrixQ_kf() {

    Q_kf << 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.001, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01;
}

void matrixR_kf() {

    R_kf <<  1,    0,    0,    0,    0,    0,    0,     0,      0,
             0,    0.5,  0,    0,    0,    0,    0,     0,      0,
             0,    0,    1,    0,    0,    0,    0,     0,      0,
             0,    0,    0,  0.5,    0,    0,    0,     0,      0,
             0,    0,    0,    0,    5,    0,    0,     0,      0,
             0,    0,    0,    0,    0,  0.1,    0,     0,      0,
             0,    0,    0,    0,    0,    0, 0.01,     0,      0,
             0,    0,    0,    0,    0,    0,    0,  0.01,      0,
             0,    0,    0,    0,    0,    0,    0,     0,   0.01;
}

// -------------- ECUACION DE PREDICCION EKF ----------------

void prediction(float U1, float U2, float U3, float U4){
    U1 = U1 + m*g;
    matrix_A_k(U1);
    __android_log_print(ANDROID_LOG_ERROR, "Empezó predicción", "xk_0 %f x_k_1 = %f",Xk_(1),X_k_1(1));
    Xk_(0)  = X_k_1(0)  + T*X_k_1(1);
    Xk_(1)  = X_k_1(1)  + T*((U1/m)*(sin(X_k_1(6))*sin(X_k_1(10)) + cos(X_k_1(6))*sin(X_k_1(8))*cos(X_k_1(10))));
    Xk_(2)  = X_k_1(2)  + T*X_k_1(3);
    Xk_(3)  = X_k_1(3)  + T*((U1/m)*(-sin(X_k_1(6))*sin(X_k_1(8))*cos(X_k_1(10)) + cos(X_k_1(6))*sin(X_k_1(10))));
    Xk_(4)  = X_k_1(4)  + T*X_k_1(5);
    Xk_(5)  = X_k_1(5)  + T*(((U1/m)*cos(X_k_1(8))*cos(X_k_1(10))) - g);
    Xk_(6)  = X_k_1(6)  + T*X_k_1(7);
    Xk_(7)  = X_k_1(7)  + T*((((Ixx-Iyy)/Izz)*X_k_1(11)*X_k_1(9)) + (U2/Izz));
    Xk_(8)  = X_k_1(8)  + T*X_k_1(9);
    Xk_(9)  = X_k_1(9)  + T*((((Izz-Ixx)/Iyy)*X_k_1(11)*X_k_1(7)) + (U3/Iyy));
    Xk_(10) = X_k_1(10) + T*X_k_1(11);
    Xk_(11) = X_k_1(11) + T*((((Iyy-Izz)/Ixx)*X_k_1(9)*X_k_1(7)) + (U4/Ixx));
    __android_log_print(ANDROID_LOG_ERROR, "Terminó predicción", "xk_0 %f x_k_1%f",Xk_(1),X_k_1(1));
}

void readZ_k(float posx, float velx, float posy, float vely, float posz, float velz, float psi, float theta, float phi){
    Z_k << posx, velx*50, posy, vely*50, posz, velz, psi, theta, phi;
}

void equations(){
    __android_log_print(ANDROID_LOG_ERROR, "Empezó equations", "X_K= %fXK_= %f x_k_1 = %f K_k0,0 %f %f %f %f %f %f %f %f",X_k(1),Xk_(1),X_k_1(1),K_k(1,0),K_k(1,1),K_k(1,2),K_k(1,3),K_k(1,4),K_k(1,5),K_k(1,6),K_k(1,7));
    Pk_ = A_k*P_k_1*A_kt + Q_kf;

/// Ganancia Optima de Kalman

    inv_ = (H*Pk_*Ht+R_kf);
    inv_ = inv_.inverse();

    K_k = Pk_*Ht*inv_;
// ------------- ECUACION DE ACTUALIZACION EKF ---------------
    inter = K_k*(Z_k - H*Xk_);
    X_k = Xk_ + inter;
    //X_k = Xk_ + K_k*(Z_k - H*Xk_);
    P_k = (w-K_k*H)*Pk_;

//-------------Modify the persitent variables----------------------
    X_k_1 = X_k;
    P_k_1 = P_k;
    __android_log_print(ANDROID_LOG_ERROR, "Terminó equations", "X_K= %fXK_= %f x_k_1 = %f K_k1,0= %f %f %f %f %f %f %f %f",X_k(1),Xk_(1),X_k_1(1), K_k(1,0),K_k(1,1),K_k(1,2),K_k(1,3),K_k(1,4),K_k(1,5),K_k(1,6),K_k(1,7));

}

VectorXf getState(){

    return X_k;
}