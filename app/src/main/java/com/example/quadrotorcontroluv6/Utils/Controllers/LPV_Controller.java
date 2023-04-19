package com.example.quadrotorcontroluv6.Utils.Controllers;

import android.content.Context;
import android.util.Log;

//import org.ejml.data.DMatrixRMaj;

public class LPV_Controller {

    public LPV_Controller(Context context) {
        iniciar_matrices_controlador();
        Log.w("LPV_controller","Iniciando LPV controller");
    }


    public void calculate_controller_stabilize(double dPhi, double dPsi, double ePsi, double eTheta, double ePhi){
        //Log.w("LPV_controller","calculate controller stabilize");
        double error_stab[] = {ePsi,eTheta,ePhi};
        calculo_controlador_stabilize(dPhi, dPsi,error_stab);
    }

    public void calculate_controller_AltHold(double Theta, double Phi, double dPhi, double dPsi, double ePsi, double eTheta, double ePhi, double eAlt){
        double error_AltHold[] = {ePsi,eTheta,ePhi,eAlt};
        calculo_controlador_AltHold(Theta, Phi, dPhi, dPsi, error_AltHold);
    }

    public void calculate_controller_Loiter(double Psi, double Theta, double Phi, double dPhi, double dPsi, double pX, double dpX, double pY, double dpY, double pZ, double dpZ, double refX, double refdX, double refY, double refdY, double refZ, double refdZ,double refPsi){
        double error_Loiter_Est[] = {refX-pX, refdX-dpX, refY-pY, refdY-dpY, refZ-pZ, refdZ-dpZ};
        double error_Loiter[] = {refX-pX, refY-pY,  refZ-pZ};
        calculo_controlador_Loiter(Psi,Theta,Phi,dPhi,dPsi,refPsi,error_Loiter_Est,error_Loiter);
    }

    public double[] getControlSignalStabilize(){

        double[] U = leer_senalesControl_stabilize();

        return U;
    }

    public double[] getControlSignalAltHold(){
        double[] U = leer_senalesControl_AltHold();
        return U;
    }

    public double[] getControlSignalLoiter(){
        double[] U = leer_senalesControl_Loiter();
        return U;
     }

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

public native void iniciar_matrices_controlador();
public native void calculo_controlador_stabilize(double dPhi, double dPsi, double error[]);
public native double[] leer_senalesControl_stabilize();
public native double[] leer_senalesControl_AltHold();
public native double[] leer_senalesControl_Loiter();

public native void calculo_controlador_AltHold(double Theta, double Phi, double dPhi, double dPsi, double error[]);
public native void calculo_controlador_Loiter(double Psi, double Theta, double Phi, double dPhi, double dPsi,double refPsi, double error_Loiter_Est[], double error_Loiter[]);
}
