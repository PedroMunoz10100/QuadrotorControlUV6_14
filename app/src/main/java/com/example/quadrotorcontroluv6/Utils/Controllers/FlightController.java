package com.example.quadrotorcontroluv6.Utils.Controllers;

import android.app.Activity;
import android.content.Context;
import android.os.BatteryManager;
import android.util.Log;

import com.example.quadrotorcontroluv6.MenuActivities.MissionActivity;
import com.example.quadrotorcontroluv6.Utils.Communication.AdkCommunicator;
import com.example.quadrotorcontroluv6.Utils.SaveFile;
import com.example.quadrotorcontroluv6.Utils.StateEstimation.AltHoldKalmanFilter;
import com.example.quadrotorcontroluv6.Utils.StateEstimation.DataCollection;
import com.example.quadrotorcontroluv6.Utils.StateEstimation.EKF;
import com.example.quadrotorcontroluv6.Utils.StateEstimation.PositionKalmanFilter;
import com.example.quadrotorcontroluv6.Utils.Controllers.LPV_Controller;

import org.ejml.data.DMatrixRMaj;
import org.ejml.equation.Equation;
import org.ejml.equation.Sequence;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import static android.content.Context.BATTERY_SERVICE;


/**
 * Created by AAstudillo on 21/09/2017.
 */

public class FlightController implements AdkCommunicator.AdbListener {

    public DataCollection mDataCollection;
    public PositionKalmanFilter posKF;
    public AdkCommunicator adkCommunicator;
    public MotorsPowers motorsPowers;
    public AltHoldKalmanFilter altHoldKF;
////********************para EKF*****************
    public EKF EKF;
    //--------------LPV controller --------------
    public LPV_Controller LPV_Controller;


    Context ctx;
    Activity act;

    DecimalFormat df = new DecimalFormat("0.000");

    BatteryManager bm;
    private int smartphoneBatLevel;
    private int batteryPercentage = 1;
    private float batteryPercentegeDec = 1;
    private float batteryCorrection = 1;
    private int batteryLecture = 1;
    private float batterySmoother = 0.2f;

    Timer controllerScheduler;
    ControllerThread controllerThread;
    double t;
    long measured_time, last_time, last_time_kf, last_time_kf_wogps, last_time_EKF, measured_time2;
    float delta_time, delta_time_kf, delta_time_kf_wogps, delta_time_EKF;

    public float[] controlSignals = new float[4];
    public float[] linearControlSignals = new float[4];

    public SaveFile mSaveFile;
    private ArrayList<String> dataList, listToSave;

    public final float QUAD_MASS = 1.568f; // 1.568 [kg]
    public final float GRAVITY = 9.807f; // [m/s^2]
    private final float I_XX = 0.0135f;
    private final float I_YY = 0.0124f;
    private final float I_ZZ = 0.0336f;
    public final float L = 0.244f; // [m]
    public final float TORQUE_DISTANCE = L*((float) Math.cos(Math.toRadians(45))); // [m]
    public final float K_T = 0.0210f;

    public float X_ref = 0f;
    public float Xdot_ref = 0f;
    public float Y_ref = 0f;
    public float Ydot_ref = 0f;
    public float Z_ref = 0f;
    public float Zdot_ref = 0f;
    public float Psi_ref = 0f;
    public float Psidot_ref = 0f;
    public float Theta_ref = 0f;
    public float Thetadot_ref = 0f;
    public float Phi_ref = 0f;
    public float Phidot_ref = 0f;

    public float Throttle = 0f; //[N]

    private double this_x, this_y, this_z, this_xdot, this_ydot, this_zdot;
    private double this_x_1, this_y_1, this_xdot_1, this_ydot_1;
    private double this_xKF, this_yKF, this_zKF, this_xdotKF, this_ydotKF, this_zdotKF;

    private String FlightMode = "nothing";

    public float[] Motor_Forces = new float[4];

    private float Thrust_Compensation;

    private PidRegulator xControl, yControl, zControl;

    private float Vx=0,Vx_1=0,Vy=0,Vy_1,x0=0,y0 =0, Vz = 0;
    private int primero=0;

    // Test controller: David-Pedro
    private DMatrixRMaj S, K, i, a, D;
    private Equation eq;

    Sequence operacion1;



    public FlightController(Context ctx, Activity act){

        this.ctx = ctx;
        this.act = act;

        mDataCollection = new DataCollection(ctx);          // Sensor data acquisition
        posKF = new PositionKalmanFilter(ctx);              // Position Kalman filter and Initial position acquisition
        //adkCommunicator = new AdkCommunicator(this, ctx);   // Communication with the Arduino Mega ADK //<----quité para pruebas
        motorsPowers = new MotorsPowers();                  // Class that contains the signals sent to the motors
        altHoldKF = new AltHoldKalmanFilter(ctx);
        //*******************EKF****************
        EKF = new EKF(ctx);
        LPV_Controller = new LPV_Controller(ctx);

        /*
        try {
            adkCommunicator.start(false);                   // Start the communication with the Arduino Mega ADK
        } catch (Exception e) {                             //<----quité para pruebas
            e.printStackTrace();
        }*/

        bm = (BatteryManager)ctx.getSystemService(BATTERY_SERVICE);
        smartphoneBatLevel = bm.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY);

        mSaveFile = new SaveFile(ctx, act);                         // Data logging class
        dataList = new ArrayList<>();
        listToSave = new ArrayList<>();

        xControl = new PidRegulator(0.0f,  0.0f,  0.0f, 0.3f, 0.0f);
        yControl = new PidRegulator(0.0f,  0.0f,  0.0f, 0.3f, 0.0f);
        zControl = new PidRegulator(0.0f,  0.0f,  0.0f, 0.3f, 0.0f);

        //Test controller: David pedro
        /*S = new DMatrixRMaj(6,1);
        K = new DMatrixRMaj(3,6);
        a = new DMatrixRMaj(3,1);
        i = new DMatrixRMaj(3,3);
        D = new DMatrixRMaj(3,1);

        fill_controller();
        eq = new Equation();
        eq.alias(S,"S", K, "K",a,"a");
        operacion1 = eq.compile("a = K*S");*/


    }

    public void acquireData(){
        /*
        if(!adkCommunicator.accessoryStarted) {
            try {
                adkCommunicator.start(false);                   // Start the communication with the Arduino Mega ADK
            } catch (Exception e) {                             //<----quité para pruebas
                e.printStackTrace();
            }
        }*/

        altHoldKF.initAltHoldKF(QUAD_MASS, I_XX, I_YY, I_ZZ);
        // **** EKF ***** ////////////////
        EKF.initEKF(QUAD_MASS,I_XX, I_YY, I_ZZ);
        Log.w("EKF","Se inicia el EKF");

        posKF.initPositionKF();                             // Initialize the Position Kalman Filter matrices
        mDataCollection.register();                         // Begins to acquire sensor data

        Log.w("Mission thread: ","Thread start");
        if (controllerScheduler != null) {                  // Check that the controllerThread is not running through the scheduler
            controllerScheduler.cancel();
        }

        dataList = new ArrayList<>();
        listToSave = new ArrayList<>();
        mSaveFile.createFile("dataFlightController");

        last_time = System.nanoTime();
        last_time_kf = last_time;
        controllerScheduler = new Timer();
        controllerThread = new ControllerThread();
        controllerScheduler.schedule(controllerThread, 0, 10);  // The controllerThread is executed each 10 ms
        t = 0;

        xControl.setCoefficients(0.04f, 0.1f, 0.1f);
        yControl.setCoefficients(0.04f, 0.1f, 0.1f);
        zControl.setCoefficients(0.04f, 0.1f, 0.1f);
    }

    public void stopAcquiring(){
        mSaveFile.closeFile();

        xControl.resetIntegrator();
        yControl.resetIntegrator();
    }

    float joystick_gain = 1;
    float joystick_gain2 = 1;
    float joystick_gain3 = 1;
    //float M_G_Throttle = 17.74f;
    float M_G_Throttle = QUAD_MASS*GRAVITY;
    //float M_G_Throttle = 18.44f;
    //float M_G_Throttle_init = 18.44f;
    float M_G_Throttle_init = QUAD_MASS*GRAVITY;

    private void ControllerExecution(){
        if(MissionActivity.armed){                          // The control outputs are only set, if the motors are armed

            if(FlightMode.equals("Stabilize")) {
                //Test controller: David-Pedro
                //S.set(vector_sensors(mDataCollection.psi,mDataCollection.psi_dot,mDataCollection.theta,mDataCollection.theta_dot,mDataCollection.phi,mDataCollection.phi_dot));
                //operacion1.perform();
                //Log.w("Controlador"," a00 = " + a.get(0,0) + " a10 = " + a.get(1,0) + " a20 = " + a.get(2,0));

                //Throttle = ((MissionActivity.mDataExchange.throttleJoystick)-50f)*0.05f; // [N] [-1, 1]
                Throttle = Throttle + ((MissionActivity.mDataExchange.throttleJoystick)-50f)*(0.005f/50);


                joystick_gain3 = joystick_gain3 + ((MissionActivity.mDataExchange.yawJoystick)-50f)*((0.005f)/50);
                if(joystick_gain3 <= 0){joystick_gain3 = 0;}

                Theta_ref = ((MissionActivity.mDataExchange.rollJoystick)-50f)*((12*3.1416f/180)/50);
                Phi_ref = ((MissionActivity.mDataExchange.pitchJoystick)-50f)*((12*3.1416f/180)/50);

                //Log.w("Joystick","theta: " + Theta_ref + " phi: " + Phi_ref);

                // LQR controller ---------------------
                //controlSignals[0] = 0f;
                //controlSignals[1] = -1.7045f*(mDataCollection.psi-Psi_ref) - 0.21785f*(mDataCollection.psi_dot-Psidot_ref);
                //controlSignals[2] = -1.5111f*(mDataCollection.theta-Theta_ref) - 0.3107f*(mDataCollection.theta_dot-Thetadot_ref);
                //controlSignals[3] = -1.6448f*(mDataCollection.phi-Phi_ref) - 0.3381f*(mDataCollection.phi_dot-Phidot_ref);
                // ------------------------------------

                // ------------LPV controller-----------------------------
                LPV_Controller.calculate_controller_stabilize(mDataCollection.phi_dot,mDataCollection.psi_dot,Psi_ref-mDataCollection.psi,Theta_ref-mDataCollection.theta,Phi_ref-mDataCollection.phi);
                controlSignals[0] = 0f;
                controlSignals[1] = (float) LPV_Controller.getControlSignalStabilize()[0];
                controlSignals[2] = (float) LPV_Controller.getControlSignalStabilize()[1];
                controlSignals[3] = (float) LPV_Controller.getControlSignalStabilize()[2];
                Log.w("Controlador LPV"," u2 = " + controlSignals[1] + " u3 = " + controlSignals[2] + " u4 = " + controlSignals[3]);

                Thrust_Compensation = (float) Math.abs((1/((Math.cos(mDataCollection.theta))*(Math.cos(mDataCollection.phi)))));

                M_G_Throttle = Throttle;  //(M_G_Throttle_init) + Throttle;     Cambio para que modo stabilize no inicie en m*g

                controlSignals[0] = (controlSignals[0] + M_G_Throttle)*Thrust_Compensation;
                //controlSignals[0] = controlSignals[0] + QUAD_MASS*GRAVITY + (Throttle);
                    // QUAD_MASS*GRAVITY is the necessary thrust to overcome gravity [N] 17.438 N
            }

            else if(FlightMode.equals("AltHold")){
                //Throttle = ((MissionActivity.mDataExchange.throttleJoystick)-50f)*0.0001f; // [m]
                Throttle = Throttle + ((MissionActivity.mDataExchange.throttleJoystick)-50f)*(0.001f/50);
                //Psi_ref = Psi_ref + ((MissionActivity.mDataExchange.yawJoystick)-50f)*((0.1f*3.1416f/180)/50);
                //if(Psi_ref <=-160*3.1416f/180){Psi_ref = -160*3.1416f/180;}
                //if(Psi_ref >=160*3.1416f/180){Psi_ref = 160*3.1416f/180;}
                //Theta_ref = ((MissionActivity.mDataExchange.rollJoystick)-50f)*((10*3.1416f/180)/50);
                //Phi_ref = ((MissionActivity.mDataExchange.pitchJoystick)-50f)*((15*3.1416f/180)/50);
                Z_ref = Z_ref + Throttle;
                Log.w("Z_ref ##############","############## Z_REF: "+Z_ref+" ##############");

                joystick_gain = joystick_gain + ((MissionActivity.mDataExchange.yawJoystick)-50f)*((0.005f)/50);
                if(joystick_gain <= 0){joystick_gain = 0;}

                //zControl.setCoefficients(0.5f*joystick_gain, 0.03f*joystick_gain2, 0.02f*joystick_gain3);
                zControl.setCoefficients(0, -0.001f*joystick_gain, 0);
                controlSignals[0] = zControl.getInput(((float)this_z)-Z_ref, 0.01f);

                // LQR controller ---------------------
                //controlSignals[0] = -1.2442f*(((float)this_z)-Z_ref) - 0.065f*(((float)this_zdot)-Zdot_ref);
                //controlSignals[0] = -1.2442f*(((float)this_z)-Z_ref) - 0.065f*(((float)this_zdot)-Zdot_ref) + controlSignals[0];
                //controlSignals[1] = -1.7045f*(mDataCollection.psi-Psi_ref) - 0.21785f*(mDataCollection.psi_dot-Psidot_ref);
                //controlSignals[2] = -1.5511f*(mDataCollection.theta-Theta_ref) - 0.3507f*(mDataCollection.theta_dot-Thetadot_ref);
                //controlSignals[3] = -1.6848f*(mDataCollection.phi-Phi_ref) - 0.3781f*(mDataCollection.phi_dot-Phidot_ref);
                // ------------------------------------
                // ------------LPV controller-----------------------------
                LPV_Controller.calculate_controller_AltHold(mDataCollection.theta,mDataCollection.phi,mDataCollection.phi_dot,mDataCollection.psi_dot,Psi_ref-mDataCollection.psi,Theta_ref-mDataCollection.theta,Phi_ref-mDataCollection.phi,Z_ref-mDataCollection.Z);
                controlSignals[0] = (float) LPV_Controller.getControlSignalAltHold()[0];
                controlSignals[1] = (float) LPV_Controller.getControlSignalAltHold()[1];
                controlSignals[2] = (float) LPV_Controller.getControlSignalAltHold()[2];
                controlSignals[3] = (float) LPV_Controller.getControlSignalAltHold()[3];
                Log.w("Controlador LPV"," u1 = " + controlSignals[0] + " u2 = " + controlSignals[1] + " u3 = " + controlSignals[2] + " u4 = " + controlSignals[3]);


                Thrust_Compensation = (float) Math.abs((1/((Math.cos(mDataCollection.theta))*(Math.cos(mDataCollection.phi)))));

                M_G_Throttle = M_G_Throttle_init;
                //controlSignals[0] = controlSignals[0] + 17.438f;
                controlSignals[0] = (controlSignals[0] + M_G_Throttle)*Thrust_Compensation;
                // QUAD_MASS*GRAVITY is the necessary thrust to overcome gravity [N]
            }
            else if(FlightMode.equals("Loiter")){
                //Throttle = ((MissionActivity.mDataExchange.throttleJoystick)-50f)*0.0001f; // [m]
                Throttle = Throttle + ((MissionActivity.mDataExchange.throttleJoystick)-50f)*(0.005f/50);
                if((Psi_ref <= 5*3.1416f/180) & (Psi_ref >= -5*3.1416f/180)){Psi_ref = 0;}
                else if(Psi_ref > 5*3.1416f/180){Psi_ref = Psi_ref - (0.003f*3.1416f/180);}
                else if(Psi_ref < -5*3.1416f/180){Psi_ref = Psi_ref + (0.003f*3.1416f/180);}

                if(Psi_ref <=-160*3.1416f/180){Psi_ref = -160*3.1416f/180;}
                else if(Psi_ref >=160*3.1416f/180){Psi_ref = 160*3.1416f/180;}

                //Psi_ref = Psi_ref + ((MissionActivity.mDataExchange.yawJoystick)-50f)*((0.1f*3.1416f/180)/50);
                Theta_ref = 0;
                Phi_ref = 0;

                //X_ref = X_ref + (((MissionActivity.mDataExchange.rollJoystick)-50f)*0.0001f); // [m]
                //Y_ref = Y_ref + (((MissionActivity.mDataExchange.pitchJoystick)-50f)*0.0001f); // [m]
                //Z_ref = Z_ref + Throttle;

                joystick_gain = joystick_gain + ((MissionActivity.mDataExchange.pitchJoystick)-50f)*((0.005f)/50);
                if(joystick_gain <= 0){joystick_gain = 0;}
                joystick_gain2 = joystick_gain2 + ((MissionActivity.mDataExchange.rollJoystick)-50f)*((0.005f)/50);
                if(joystick_gain2 <= 0){joystick_gain2 = 0;}
                joystick_gain3 = joystick_gain3 + ((MissionActivity.mDataExchange.yawJoystick)-50f)*((0.005f)/50);
                if(joystick_gain3 <= 0){joystick_gain3 = 0;}

                //xControl.setCoefficients(0.04f*joystick_gain, 0.1f, 0.1f*joystick_gain2);
                //yControl.setCoefficients(0.04f*joystick_gain, 0.1f, 0.1f*joystick_gain2);

                //Theta_ref = xControl.getInput(X_ref, (float)this_x, 0.01f);
                //Phi_ref = yControl.getInput(Y_ref, (float)this_y, 0.01f);

                // LQR controller ---------------------
                //controlSignals[0] = -(1.2542f*joystick_gain3)*(((float)this_z)-Z_ref) - (0.075f)*(((float)this_zdot)-Zdot_ref);
                //controlSignals[1] = -(1.7045f)*(mDataCollection.psi-Psi_ref) - (0.21785f)*(mDataCollection.psi_dot-Psidot_ref);
                //controlSignals[2] = -1.5111f*(mDataCollection.theta-Theta_ref) - 0.3107f*(mDataCollection.theta_dot-Thetadot_ref);
                //controlSignals[3] = -1.6448f*(mDataCollection.phi-Phi_ref) - 0.3381f*(mDataCollection.phi_dot-Phidot_ref);

                //controlSignals[2] = controlSignals[2] - ((0.315612f*joystick_gain)*(((float)this_xKF)-X_ref)) - ((0.01119f*joystick_gain2)*(((float)this_xdotKF)-Xdot_ref));
                //controlSignals[3] = controlSignals[3] - ((0.336130f*joystick_gain)*(((float)this_yKF)-Y_ref)) - ((0.01124f*joystick_gain2)*(((float)this_ydotKF)-Ydot_ref));
                // ------------------------------------
                // -----------------------LPV controller --------------
                LPV_Controller.calculate_controller_Loiter(mDataCollection.psi,mDataCollection.theta,mDataCollection.phi,mDataCollection.phi_dot,mDataCollection.psi_dot,mDataCollection.X,mDataCollection.X_dot,mDataCollection.Y,mDataCollection.Y_dot,mDataCollection.Z,mDataCollection.Z_dot,X_ref,Xdot_ref,Y_ref,Ydot_ref,Z_ref,Zdot_ref,Psi_ref);
                controlSignals[0] = (float) LPV_Controller.getControlSignalLoiter()[0];
                controlSignals[1] = (float) LPV_Controller.getControlSignalLoiter()[1];
                controlSignals[2] = (float) LPV_Controller.getControlSignalLoiter()[2];
                controlSignals[3] = (float) LPV_Controller.getControlSignalLoiter()[3];
                Log.w("Controlador LPV Loiter"," u1 = " + controlSignals[0] + " u2 = " + controlSignals[1] + " u3 = " + controlSignals[2] + " u4 = " + controlSignals[3]);


                Thrust_Compensation = (float) Math.abs(1/((Math.cos(mDataCollection.theta))*(Math.cos(mDataCollection.phi))));

                M_G_Throttle = M_G_Throttle_init + Throttle;
                //controlSignals[0] = controlSignals[0] + 17.438f;
                controlSignals[0] = (controlSignals[0] + M_G_Throttle)*Thrust_Compensation;
                // QUAD_MASS*GRAVITY is the necessary thrust to overcome the gravity [N]
            }
            /*
            else if(FlightMode.equals("RTL")){

            }
            else if(FlightMode.equals("Auto")){

            }
            else if(FlightMode.equals("Land")){

            }
            */
            else { // If armed without any flight mode, just turn on the motors
                Throttle = ((MissionActivity.mDataExchange.throttleJoystick)-50f)*0.001f; // [N] [-1, 1]
                if (controlSignals[0] <= QUAD_MASS*GRAVITY*0.9f){
                    controlSignals[0] = controlSignals[0]+Throttle;
                    controlSignals[1] = 0;
                    controlSignals[2] = 0;
                    controlSignals[3] = 0;
                }
                if(controlSignals[0] > QUAD_MASS*GRAVITY*0.9f){
                    controlSignals[0] = QUAD_MASS*GRAVITY*0.9f;
                }
            }

            //setControlOutputs(controlSignals[0],controlSignals[1],controlSignals[2],controlSignals[3]); //<----quité para pruebas
        }
        else{
            turnOffMotors();
        }
    }

    private void setControlOutputs(float u, float tau_psi, float tau_theta, float tau_phi){

        Motor_Forces[0] = 0.2500f*u - 11.9048f*tau_psi - 1.4490f*tau_theta - 1.4490f*tau_phi; // [N] 11.9048*tau_psi
        Motor_Forces[1] = 0.2500f*u + 11.9048f*tau_psi - 1.4490f*tau_theta + 1.4490f*tau_phi; // [N]
        Motor_Forces[2] = 0.2500f*u - 11.9048f*tau_psi + 1.4490f*tau_theta + 1.4490f*tau_phi; // [N]
        Motor_Forces[3] = 0.2500f*u + 11.9048f*tau_psi + 1.4490f*tau_theta - 1.4490f*tau_phi; // [N]

        motorsPowers.m1 = (int)(-1.983f* Math.pow(Motor_Forces[0],2) + 47.84f*Motor_Forces[0] + 3.835f); // [0, 255]
        motorsPowers.m2 = (int)((-1.983f* Math.pow(Motor_Forces[1],2) + 47.84f*Motor_Forces[1] + 3.835f)*0.975f); // [0, 255]
        motorsPowers.m3 = (int)(-1.983f* Math.pow(Motor_Forces[2],2) + 47.84f*Motor_Forces[2] + 3.835f); // [0, 255]
        motorsPowers.m4 = (int)((-1.983f* Math.pow(Motor_Forces[3],2) + 47.84f*Motor_Forces[3] + 3.835f)*0.975f); // [0, 255]

        /*motorsPowers.m1 = (int)(motorsPowers.m1 * batteryCorrection);
        motorsPowers.m2 = (int)(motorsPowers.m2 * batteryCorrection);
        motorsPowers.m3 = (int)(motorsPowers.m3 * batteryCorrection);
        motorsPowers.m4 = (int)(motorsPowers.m4 * batteryCorrection);*/


        if(motorsPowers.m1 > 255){motorsPowers.m1 = 255;} // Motors saturation
        if(motorsPowers.m1 < 0){motorsPowers.m1 = 0;}
        if(motorsPowers.m2 > 255){motorsPowers.m2 = 255;}
        if(motorsPowers.m2 < 0){motorsPowers.m2 = 0;}
        if(motorsPowers.m3 > 255){motorsPowers.m3 = 255;}
        if(motorsPowers.m3 < 0){motorsPowers.m3 = 0;}
        if(motorsPowers.m4 > 255){motorsPowers.m4 = 255;}
        if(motorsPowers.m4 < 0){motorsPowers.m4 = 0;}

        //adkCommunicator.setPowers(motorsPowers); //<----quité para pruebas

    }

    private void turnOffMotors(){
        controlSignals[0] = 0;
        controlSignals[1] = 0;
        controlSignals[2] = 0;
        controlSignals[3] = 0;

        motorsPowers.m1 = 0;
        motorsPowers.m2 = 0;
        motorsPowers.m3 = 0;
        motorsPowers.m4 = 0;

        //adkCommunicator.setPowers(motorsPowers);    //<---comenté esto para pruebas
    }

    @Override
    public void onBatteryVoltageArrived(float batteryVoltage){ //It's executed when Android receives the Battery data from ADK
        //this.batteryPercentage = (int)(batteryVoltage*66.6667 - 740); //12.6 V full -- 11.1 V empty
        this.batteryLecture = (int)(batteryVoltage*31.25 - 293.75); //12.6 V full -- 9.4 V empty
        if(this.batteryPercentage < 5) {
            this.batteryPercentage = this.batteryLecture;
        }else{
            this.batteryPercentage = (int)(batteryPercentage*batterySmoother + batteryLecture*(1-batterySmoother));
        }
        batteryPercentegeDec = batteryPercentage/100;

        this.batteryCorrection = (float)(0.9f*((-0.0723f*(Math.pow(batteryPercentegeDec,2))) - (0.428f*batteryPercentegeDec) + 1.4605f));
        if(batteryCorrection < 1){batteryCorrection = 1;}
        else if(batteryCorrection > 1.3){batteryCorrection = 1.3f;}

        this.smartphoneBatLevel = bm.getIntProperty(BatteryManager.BATTERY_PROPERTY_CAPACITY);
        MissionActivity.UIHandler.post(new Runnable() {
            @Override
            public void run() {
                MissionActivity.tv_quadbatt.setText(batteryPercentage + " %");
                MissionActivity.tv_smartbatt.setText(smartphoneBatLevel + " %");
            }
        });
    }

    public void turnLed(boolean on){
        if(on){adkCommunicator.commTest(1,0,0,0);}
        else{adkCommunicator.commTest(0,0,0,0);}
    }

    public void changeFlightMode(String flightMode){
        FlightMode = flightMode;
        if(flightMode.equals("")){

        }
        else if(flightMode.equals("Stabilize")){
            Psi_ref = mDataCollection.psi;
            Theta_ref = 0;
            Phi_ref = 0;
        }
        else if(flightMode.equals("AltHold")){
            Z_ref = (float)this_z + 0.3f;
            Psi_ref = mDataCollection.psi;
            Theta_ref = 0;
            Phi_ref = 0;

            joystick_gain = 1;
            joystick_gain2 = 1;

            zControl.setCoefficients(0f, -0.03f, 0);
            zControl.resetIntegrator();
        }
        else if(flightMode.equals("Loiter")){
            X_ref = (float)this_x;
            Y_ref = (float)this_y;
            Z_ref = (float)this_z + 0.5f;
            Psi_ref = mDataCollection.psi;
            Theta_ref = 0;
            Phi_ref = 0;

            joystick_gain = 1;
            joystick_gain2 = 1;

            xControl.setCoefficients(0.04f, 0.1f, 0.1f);
            yControl.setCoefficients(0.04f, 0.1f, 0.1f);
            xControl.resetIntegrator();
            yControl.resetIntegrator();
        }
        else if(flightMode.equals("RTL")){
            X_ref = (float)this_x;
            Y_ref = (float)this_y;
            Z_ref = (float)this_z;
            Psi_ref = 0;
            Theta_ref = 0;
            Phi_ref = 0;
        }
        else if(flightMode.equals("Auto")){
            X_ref = (float)this_x;
            Y_ref = (float)this_y;
            Z_ref = (float)this_z;
            Psi_ref = 0;
            Theta_ref = 0;
            Phi_ref = 0;
        }
        else if(flightMode.equals("Land")){
            X_ref = (float)this_x;
            Y_ref = (float)this_y;
            Z_ref = (float)this_z;
            Psi_ref = 0;
            Theta_ref = 0;
            Phi_ref = 0;
        }
    }

            public double last_timeControl, delta_time_Control;
    private class ControllerThread extends TimerTask {
        @Override
        public void run() {

            measured_time = System.nanoTime();
            delta_time = ((float) (measured_time - last_time)) / 1000000.0f; // [ms].;
            last_time = measured_time;
            Log.w("controlled therad", "last time = " + last_time);
            t = t + delta_time;     // [ms];
            delta_time_kf = (measured_time - last_time_kf) / 1000000000; // [s].;
            delta_time_kf_wogps = (measured_time - last_time_kf_wogps) / 1000000000; // [s].;

            estimateQuadrotorStates();

            ControllerExecution();

            last_timeControl = System.nanoTime();
            delta_time_Control = ((float) (last_timeControl-measured_time)) / 1000000.0f; // [ms].;

            double[] datosEKF = new double[12];
            double[] datosaltHoldKF = new double[12];
            //datosEKF = EKF.getEstimatedState();
            datosaltHoldKF = altHoldKF.getEstimatedState();
            //Log.w("EKF", " x= " + datosEKF[0] + " y= " + datosEKF[2] + " z= " + datosEKF[4] + " psi= " + datosEKF[6]);
            //Log.w("Altidude Hold", "dato 0=" + datosaltHoldKF[0]);

            mSaveFile.writeDatainFile(df.format(delta_time_Control) + System.lineSeparator());
            //mSaveFile.writeDatainFile(df.format(t) + "." + df.format(delta_time) + /*2*/
            //        "." + df.format(this_x) + "." + df.format(this_y) + "." + df.format(this_z) + /*5*/
            //        "." + df.format(MissionActivity.quadrotorState[3])+ "." + df.format(MissionActivity.quadrotorState[4]) + "." + df.format(MissionActivity.quadrotorState[5]) + /*8*/
            //        "." + df.format(controlSignals[0]) + "." + df.format(controlSignals[1]) + "." + df.format(controlSignals[2]) + "." + df.format(controlSignals[3]) + /*12*/
            //        "." + motorsPowers.m1 + "." + motorsPowers.m2 + "." + motorsPowers.m3 + "." + motorsPowers.m4 + /*16*/
            //        "." + MissionActivity.quadrotorState[6] + "." + MissionActivity.quadrotorState[7] + "." + df.format(Throttle) + /*19*/
            //        "." + df.format(X_ref) + "." + df.format(Y_ref) + "." + df.format(Z_ref) + "." + df.format(Psi_ref) + "." + df.format(Theta_ref) + "." + df.format(Phi_ref) + /*25*/
            //        "." + df.format(this_xdot) + "." + df.format(this_ydot) + "." + df.format(this_zdot) + /*28*/
            //        "." + df.format(mDataCollection.earthAccVals[0]) + "." + df.format(mDataCollection.earthAccVals[1]) + "." + df.format(mDataCollection.earthAccVals[2]) + /*31*/

            //        "." + df.format(altHoldKF.getEstimatedState()[0]) + "." + df.format(altHoldKF.getEstimatedState()[1]) + /*32-33*/
            //        "." + df.format(altHoldKF.getEstimatedState()[2]) + "." + df.format(altHoldKF.getEstimatedState()[3]) +
            //        "." + df.format(altHoldKF.getEstimatedState()[4]) + "." + df.format(altHoldKF.getEstimatedState()[5]) +
            //        "." + df.format(altHoldKF.getEstimatedState()[6]) + "." + df.format(altHoldKF.getEstimatedState()[7]) +
            //        "." + df.format(altHoldKF.getEstimatedState()[8]) + "." + df.format(altHoldKF.getEstimatedState()[9]) +
            //        "." + df.format(altHoldKF.getEstimatedState()[10]) + "." + df.format(altHoldKF.getEstimatedState()[11]) + /*43*/
////------------------------------------------------------------------------------------------------------------------------------
                    /* "," + df.format(datosaltHoldKF[0]) + "," + df.format(datosaltHoldKF[1]) +
                    "," + df.format(datosaltHoldKF[2]) + "," + df.format(datosaltHoldKF[3]) +
                    "," + df.format(datosaltHoldKF[4]) + "," + df.format(datosaltHoldKF[5]) +
                    "," + df.format(datosaltHoldKF[6]) + "," + df.format(datosaltHoldKF[7]) +
                    "," + df.format(datosaltHoldKF[8]) + "," + df.format(datosaltHoldKF[9]) +
                    "," + df.format(datosaltHoldKF[10]) + "," + df.format(datosaltHoldKF[11]) + */
//________________________________________________________________________________________________________________________________
            //        "." + df.format(datosEKF[0]) + "." + df.format(datosEKF[1]) +
            //        "." + df.format(datosEKF[2]) + "." + df.format(datosEKF[3]) +
            //        "." + df.format(datosEKF[4]) + "." + df.format(datosEKF[5]) +
            //        "." + df.format(datosEKF[6]) + "." + df.format(datosEKF[7]) +
            //        "." + df.format(datosEKF[8]) + "." + df.format(datosEKF[9]) +
            //        "." + df.format(datosEKF[10]) + "." + df.format(datosEKF[11]) + /*55*/
            //        "." + df.format(mDataCollection.conv_x) + "." + df.format(mDataCollection.conv_y) + "." + mDataCollection.baroElevation +/*56-58*/
            //        "." + mDataCollection.gps_longitude + "." + mDataCollection.gps_latitude +
            //        "." + df.format(mDataCollection.psi) + "." + df.format(mDataCollection.theta) + "." + df.format(mDataCollection.phi) + System.lineSeparator());// +

                    //"," + joystick_gain + "," + joystick_gain2 + "," + joystick_gain3 + "," + df.format(controlSignals[0]) + "," + df.format(Thrust_Compensation) +
                    //"," + batteryCorrection + System.lineSeparator());
            editGUI();//posición 44 empieza la X
        }

        private void estimateQuadrotorStates(){
            if(delta_time_kf >= 0.02){ //delta_time_kf >= 0.05
                //posKF.executePositionKF(0, 0, mDataCollection.baroElevation,mDataCollection.earthAccVals[0],mDataCollection.earthAccVals[1],mDataCollection.earthAccVals[2]);
                posKF.executePositionKF(mDataCollection.conv_x,mDataCollection.conv_y,mDataCollection.baroElevation,mDataCollection.earthAccVals[0],mDataCollection.earthAccVals[1],mDataCollection.earthAccVals[2]);
                this_x = posKF.getEstimatedState()[0];
                this_y = posKF.getEstimatedState()[1];
                this_z = mDataCollection.baroElevation;
                this_xdot = this_xdotKF;
                this_ydot = this_ydotKF;
                this_zdot = posKF.getEstimatedState()[5];

                last_time_kf = measured_time;
                this_x_1 = this_x;
                this_y_1 = this_y;
            }
            else{
                posKF.executePositionKF_woGPS(mDataCollection.earthAccVals[0],mDataCollection.earthAccVals[1],mDataCollection.earthAccVals[2]);
                this_x = posKF.getEstimatedState_woGPS()[0];
                this_y = posKF.getEstimatedState_woGPS()[1];
                this_z = mDataCollection.baroElevation;
                //this_xdot = (this_x - this_x_1)/delta_time_kf_wogps;
                //this_ydot = (this_y - this_y_1)/delta_time_kf_wogps;
                this_xdot = this_xdotKF;
                this_ydot = this_ydotKF;
                if(Math.abs((this_x - this_x_1)/delta_time_kf_wogps) < 0.02){
                    this_xdot = 0;
                }
                if(Math.abs((this_y - this_y_1)/delta_time_kf_wogps) < 0.02){
                    this_ydot = 0;
                }
                this_zdot = posKF.getEstimatedState_woGPS()[5];

                last_time_kf_wogps = measured_time;
                this_x_1 = this_x;
                this_y_1 = this_y;
                this_xdot_1 = this_xdot;
                this_ydot_1 = this_ydot;
            }

            if (primero == 0){
                x0 = (float)mDataCollection.conv_x;
                y0 = (float)mDataCollection.conv_y;
                primero = 1;
            }

            //EKF.executeEKF((float)mDataCollection.conv_x-x0, Vx,(float)mDataCollection.conv_y-y0, Vy,mDataCollection.baroElevation, Vz,mDataCollection.psi,mDataCollection.theta,mDataCollection.phi, controlSignals[0],controlSignals[1],controlSignals[2],controlSignals[3]);

           // EKF.executeEKF((float)mDataCollection.conv_x, Vx,(float)mDataCollection.conv_y, Vy,mDataCollection.baroElevation,mDataCollection.psi,mDataCollection.theta,mDataCollection.phi, controlSignals[0],controlSignals[1],controlSignals[2],controlSignals[3]);
            //EKF.executeEKF((float)mDataCollection.conv_x,(float)mDataCollection.conv_y,mDataCollection.baroElevation,mDataCollection.psi,mDataCollection.theta,mDataCollection.phi, 13,0,0,0);

            Vx = (float) (mDataCollection.earthAccVals[0]*0.01);
            Vy = (float) (mDataCollection.earthAccVals[1]*0.01);
            Vz = (float) (mDataCollection.earthAccVals[2]*0.01);
            //Vx_1 = Vx;
            //Vy_1 = Vy;

            //Log.w("Velocidad","VelX: "+Vx+" VelY: "+Vy);

            //measured_time2 = System.nanoTime();
            //delta_time_EKF = ((float) (measured_time - measured_time2)) / 1000000.0f; // [ms].;
            Log.w("EKF señales U"," U1= "+controlSignals[0]+" U2= "+controlSignals[1]+" U3= "+controlSignals[2]+ " U4= "+controlSignals[3]);
            altHoldKF.executeAltHoldKF((float)this_x, (float)this_y, mDataCollection.baroElevation,mDataCollection.psi,mDataCollection.theta,mDataCollection.phi,((controlSignals[0]/Thrust_Compensation)-M_G_Throttle),controlSignals[1],controlSignals[2],controlSignals[3]);
            // TODO: Test this KF
            // TODO: Add integral part of LQI control
            /*this_xKF = altHoldKF.getEstimatedState()[0];
            this_xdotKF = altHoldKF.getEstimatedState()[1];
            this_yKF = altHoldKF.getEstimatedState()[2];
            this_ydotKF = altHoldKF.getEstimatedState()[3];
            this_zKF = altHoldKF.getEstimatedState()[4];
            this_zdotKF = altHoldKF.getEstimatedState()[5];*/
            setQuadrotorState();
        }

        private void setQuadrotorState(){
            MissionActivity.quadrotorState[0] = (float)this_x;          // x [m]
            MissionActivity.quadrotorState[1] = (float)this_y;          // y [m]
            MissionActivity.quadrotorState[2] = (float)this_z;          // z [m]
            MissionActivity.quadrotorState[3] = mDataCollection.theta;  // roll [rad]
            MissionActivity.quadrotorState[4] = mDataCollection.phi;    // pitch [rad]
            MissionActivity.quadrotorState[5] = mDataCollection.psi;    // yaw [rad]
            MissionActivity.quadrotorState[6] = batteryPercentage;      // quadrotor battery [%]
            MissionActivity.quadrotorState[7] = smartphoneBatLevel;     // smartphone battery [%]
        }

        private void editGUI(){
            MissionActivity.UIHandler.post(new Runnable() {
                @Override
                public void run() {
                    if(MissionActivity.armed){
                        MissionActivity.tv_east.setText(df.format(MissionActivity.quadrotorState[0]));
                        MissionActivity.tv_north.setText(df.format(MissionActivity.quadrotorState[1]));
                        MissionActivity.tv_elevation.setText(df.format(MissionActivity.quadrotorState[2]));
                    }
                    MissionActivity.tv_roll.setText(df.format(mDataCollection.orientationValsDeg[2]) + " °");
                    MissionActivity.tv_pitch.setText(df.format(mDataCollection.orientationValsDeg[1]) + " °");
                    MissionActivity.tv_yaw.setText(df.format(mDataCollection.orientationValsDeg[0]) + " °");
                    MissionActivity.tv_dt.setText(df.format(delta_time)+" ms");
                    MissionActivity.pb_rolljoystick.setProgress(MissionActivity.mDataExchange.rollJoystick);
                    MissionActivity.pb_pitchjoystick.setProgress(MissionActivity.mDataExchange.pitchJoystick);
                    MissionActivity.pb_yawjoystick.setProgress(MissionActivity.mDataExchange.yawJoystick);
                    MissionActivity.pb_throttlejoystick.setProgress(MissionActivity.mDataExchange.throttleJoystick);
                    MissionActivity.pb_motor1.setProgress(motorsPowers.m1*100/255);
                    MissionActivity.pb_motor2.setProgress(motorsPowers.m2*100/255);
                    MissionActivity.pb_motor3.setProgress(motorsPowers.m3*100/255);
                    MissionActivity.pb_motor4.setProgress(motorsPowers.m4*100/255);
                }
            });
        }
    }


    public static class MotorsPowers
    {
        public int m1, m2, m3, m4;  // 0-1023 (10 bits values).

        public int getMean()
        {
            return (m1+m2+m3+m4) / 4;
        }
    }

    public DMatrixRMaj vector_sensors(double psi,double dpsi, double theta, double dtheta, double phi, double dphi){
        DMatrixRMaj datos = new DMatrixRMaj(6,1);
        datos.set(0,0,psi);
        datos.set(1,0,dpsi);
        datos.set(2,0,theta);
        datos.set(3,0,dtheta);
        datos.set(4,0,phi);
        datos.set(5,0,dphi);
        return datos;
    }

    public void fill_controller(){
        K.set(0,0,-7.4847);
        K.set(1,0,0);
        K.set(2,0,0);
        K.set(0,1,-1.3421);
        K.set(1,1,0);
        K.set(2,1,0);
        K.set(0,2,0);
        K.set(1,2,-6.2886);
        K.set(2,2,0);
        K.set(0,3,0);
        K.set(1,3,-0.8092);
        K.set(2,3,0);
        K.set(0,4,0);
        K.set(1,4,0);
        K.set(2,4,-6.2886);
        K.set(0,5,0);
        K.set(1,5,0);
        K.set(2,5,-0.8092);

        i.set(0,0,0.2835);
        i.set(1,0,0);
        i.set(2,0,0);
        i.set(0,1,0);
        i.set(1,1,0.2755);
        i.set(2,1,0);
        i.set(0,2,0);
        i.set(1,2,0);
        i.set(2,2,0.2755);

    }

    public void fill_references(double psi, double tetha, double phi){
        D.set(0,0,0.2835);
        D.set(1,0,0);
        D.set(2,0,0);
    }
}
