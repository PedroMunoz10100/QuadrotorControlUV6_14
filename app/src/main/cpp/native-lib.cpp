#include <jni.h>
#include <string>
#include <android/log.h>
#include "EKFilter.h"   // Incluye el archivo "EKFilter" al codigo

#include "LPV_Calculos.h"
//#include "MatricesLPV_AltHold.h"

using namespace Eigen;  // Declaracion del uso del espacio de nombres para palabras reservadas

jfloat sumita(float a, float b);

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_quadrotorcontroluv6_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}


extern "C"
JNIEXPORT void JNICALL
Java_com_example_quadrotorcontroluv6_Utils_StateEstimation_EKF_setState(JNIEnv *env, jobject instance,
                                                                        jdoubleArray ic_,
                                                                        jdouble Q_val,
                                                                        jdouble R_val, jdouble m,
                                                                        jdouble ixx, jdouble iyy,
                                                                        jdouble izz) {
    jdouble *ic = env->GetDoubleArrayElements(ic_, NULL);
    //double ic[] = {x_ic, 0 , y_ic, 0, z_ic , 0 , psi_ic , 0 , theta_ic , 0 , phi_ic , 456.89 };
    //__android_log_print(ANDROID_LOG_ERROR, "TRACKERS desde native", "%f",ic_[11]);
    initPositionEKF(ic, Q_val, R_val, m, Ixx, Iyy, Izz);

    //env->ReleaseDoubleArrayElements(ic_, ic, 0);
}extern "C"
JNIEXPORT void JNICALL
Java_com_example_quadrotorcontroluv6_Utils_StateEstimation_EKF_calculateEKF(JNIEnv *env,
                                                                            jobject instance,
                                                                            jfloat posx,
                                                                            jfloat velx,
                                                                            jfloat posy,
                                                                            jfloat vely,
                                                                            jfloat posz,
                                                                            jfloat velz, jfloat psi,
                                                                            jfloat theta,
                                                                            jfloat phi, jfloat u,
                                                                            jfloat tau_psi,
                                                                            jfloat tau_theta,
                                                                            jfloat tau_phi) {
    readZ_k(posx, velx, posy, vely, posz, velz, psi, theta, phi);
    prediction(u, tau_psi, tau_theta, tau_phi);
    equations();
}


extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_quadrotorcontroluv6_Utils_StateEstimation_EKF_getState(JNIEnv *env, jobject instance) {
    VectorXf estados = getState();
    double esta2[12];
    for (int i = 0; i < 12; i ++){
        esta2[i] = estados(i);
    }


    jdoubleArray doubleArray = env->NewDoubleArray(12);
    env->SetDoubleArrayRegion(doubleArray, 0, 12, (const jdouble*) esta2 );

    //esta1[1] =2;

    return doubleArray;
}

extern "C"
JNIEXPORT jstring JNICALL
Java_com_example_quadrotorcontroluv6_Utils_StateEstimation_EKF_string(JNIEnv *env, jobject instance) {
    float c = 456;
    //std::string hello = "Hello from C++";
    char output[40];
    sprintf(output, "la suma es %f de cualquier cosa", c);
    return env->NewStringUTF(output);
}

//--------------------------------Pruebas LPV --------------------------
extern "C"
JNIEXPORT void JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_calculo_1controlador_1stabilize(
        JNIEnv *env, jobject thiz, jdouble dPhi, jdouble dPsi, jdoubleArray error_) {
    // TODO: implement calculo_controlador_stabilize()
    jdouble *error = env->GetDoubleArrayElements(error_, NULL);
    calculando_controlador_stabilize(dPhi,dPsi,error);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_iniciar_1matrices_1controlador(
        JNIEnv *env, jobject thiz) {
    // TODO: implement iniciar_matrices_controlador()
    asignar_matrices_controlador();
}

extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_leer_1senalesControl_1stabilize(
        JNIEnv *env, jobject thiz) {
    // TODO: implement leer_senalesControl_stabilize()
    VectorXf U_stab_a = leer_U_stabilize();
    double U_stab[3];
    U_stab[0] = U_stab_a(0);
    U_stab[1] = U_stab_a(1);
    U_stab[2] = U_stab_a(2);

    jdoubleArray doubleArray = env->NewDoubleArray(3);
    env->SetDoubleArrayRegion(doubleArray, 0, 3, (const jdouble*) U_stab );
    return doubleArray;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_calculo_1controlador_1AltHold(
        JNIEnv *env, jobject thiz, jdouble theta, jdouble phi, jdouble d_phi, jdouble d_psi,
        jdoubleArray error_) {
    // TODO: implement calculo_controlador_AltHold()
    jdouble *error = env->GetDoubleArrayElements(error_, NULL);
    calculando_controlador_AltHold(theta,phi,d_phi,d_psi,error);
}
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_leer_1senalesControl_1AltHold(
        JNIEnv *env, jobject thiz) {
    // TODO: implement leer_senalesControl_AltHold()
    VectorXf U_AltHold_a = leer_U_AltHold();
    double U_AltHold[4];
    U_AltHold[0] =  U_AltHold_a(0);
    U_AltHold[1] =  U_AltHold_a(1);
    U_AltHold[2] =  U_AltHold_a(2);
    U_AltHold[3] =  U_AltHold_a(3);

    jdoubleArray doubleArray = env->NewDoubleArray(4);
    env->SetDoubleArrayRegion(doubleArray, 0, 4, (const jdouble*) U_AltHold );
    return doubleArray;
}
extern "C"
JNIEXPORT void JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_calculo_1controlador_1Loiter(
        JNIEnv *env, jobject thiz, jdouble psi, jdouble theta, jdouble phi, jdouble d_phi,
        jdouble d_psi, jdouble ref_psi, jdoubleArray error__loiter__est_,
        jdoubleArray error__loiter_) {
    // TODO: implement calculo_controlador_Loiter()
    jdouble *error__loiter__est = env->GetDoubleArrayElements(error__loiter__est_, NULL);
    jdouble *error__loiter = env->GetDoubleArrayElements(error__loiter_, NULL);
    calculando_controlador_Loiter(psi,theta,phi,d_phi,d_psi,ref_psi,error__loiter__est,error__loiter);
}
extern "C"
JNIEXPORT jdoubleArray JNICALL
Java_com_example_quadrotorcontroluv6_Utils_Controllers_LPV_1Controller_leer_1senalesControl_1Loiter(
        JNIEnv *env, jobject thiz) {
    // TODO: implement leer_senalesControl_Loiter()
    VectorXf U_Loiter_a = leer_U_Loiter();
    double U_Loiter[4];
    U_Loiter[0] =  U_Loiter_a(0);
    U_Loiter[1] =  U_Loiter_a(1);
    U_Loiter[2] =  U_Loiter_a(2);
    U_Loiter[3] =  U_Loiter_a(3);

    jdoubleArray doubleArray = env->NewDoubleArray(4);
    env->SetDoubleArrayRegion(doubleArray, 0, 4, (const jdouble*) U_Loiter );
    return doubleArray;
}