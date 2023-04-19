
package com.example.quadrotorcontroluv6.Utils.StateEstimation;

import android.content.Context;
import android.util.Log;

import org.ejml.data.DMatrixRMaj;

/**
 * Created by AAstudillo on 11/05/2018.
 */

public class EKF {
    public InitialConditions mInitialConditions = null;

    private static final double dt = 0.01; //Our sample time
    private static final double Q_val = 0.1;
    private static final double R_val = 1;
    public double x_ic, y_ic, z_ic, psi_ic, theta_ic, phi_ic = 0;
    public double Mass, Izz, Iyy, Ixx;

    double[] ic, x_hat;
    DMatrixRMaj xhat_k_1, P_k_1, A, B, Q, Ro, H, z, U;
    //EKFTotal f;

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
    }

    public EKF(Context context) {
        mInitialConditions = new InitialConditions(context);
        mInitialConditions.acquireIC();
    }

    public void initEKF(double mass, double i_xx, double i_yy, double i_zz){
        Mass = mass;
        Izz = i_zz;
        Iyy = i_yy;
        Ixx = i_xx;

        /*x_ic = mInitialConditions.getx_ic();
        y_ic = mInitialConditions.gety_ic();
        z_ic = mInitialConditions.getz_ic();*/
        x_ic = 0;
        y_ic = 0;
        z_ic = mInitialConditions.getz_ic();
        /*psi_ic = mInitialConditions.getpsi_ic();
        theta_ic = mInitialConditions.gettheta_ic();
        phi_ic = mInitialConditions.getphi_ic();*/

        //Log.w("estados iniciales",x_ic + " " + y_ic + " " + z_ic + " " + psi_ic + " " + theta_ic+ " " + phi_ic);
        // f = new EKFOperationsTotal();

        ic = new double[]{x_ic, 0 , y_ic, 0, z_ic , 0 , psi_ic , 0 , theta_ic , 0 , phi_ic , 0 };
        //ic = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        //xhat_k_1 = new DMatrixRMaj(12, 1, true, ic);
        //P_k_1 = new DMatrixRMaj(12,12); //9x9 matrix of zeros

        //A = new DMatrixRMaj(12, 12);
        //Q = createQ(dt*Q_val);
        //Ro = createR(R_val); // full
        //H = createH(); //full
        //z = new DMatrixRMaj(4, 1); //4x1 vector of zeros
        x_hat = new double[12];

        Log.w("EKF", "HOLITA");
        String hh = string();
        Log.w("EKF",hh);
        // f.configure(A,Q,H);
        //setState(x_ic, y_ic, z_ic, psi_ic, theta_ic, phi_ic,dt*Q_val,R_val);
        setState(ic,dt*Q_val,R_val, Mass,  Ixx, Iyy, Izz);
        //String hh1 = setState(x_ic, y_ic, z_ic, psi_ic, theta_ic, phi_ic,dt*Q_val,R_val);
        //Log.w("EKF",hh1);
            }

    public void executeEKF(float posx, float velx, float posy, float vely, float posz, float velz, float psi, float theta, float phi, float u, float tau_psi, float tau_theta, float tau_phi){

        calculateEKF(posx, velx, posy, vely, posz, velz, psi, theta, phi, u, tau_psi, tau_theta, tau_phi);

    }

    public double[] getEstimatedState(){

        x_hat = getState();
        return x_hat;
    }

    //public native double MULTI();
    //public native void setState(double x_ic, double y_ic,double z_ic , double psi_ic , double theta_ic , double phi_ic, double Q_val, double R_val);
    //public native String setState(double x_ic, double y_ic,double z_ic , double psi_ic , double theta_ic , double phi_ic, double Q_val, double R_val);
    public native void setState(double[] ic, double Q_val, double R_val, double m,  double Ixx, double Iyy, double Izz);
    public native void calculateEKF(float posx, float velx, float posy, float vely, float posz, float velz, float psi, float theta, float phi, float u, float tau_psi, float tau_theta, float tau_phi);
    public native double[] getState();
    public native String string();
}