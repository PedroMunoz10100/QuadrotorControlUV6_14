package com.example.quadrotorcontroluv6.MenuActivities;

import android.content.Intent;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;

import com.example.quadrotorcontroluv6.R;
import com.example.quadrotorcontroluv6.TestActivities.AttitudeKFTest;
import com.example.quadrotorcontroluv6.TestActivities.MotorsTestActivity;
import com.example.quadrotorcontroluv6.TestActivities.PositionKFTest;
import com.example.quadrotorcontroluv6.TestActivities.TcpTest;



public class TestsActivity extends AppCompatActivity {

    private Button bt_attKF, bt_posKF, bt_motorTest, bt_commTest, bt_attControl;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_tests);


        bt_attKF = (Button) findViewById(R.id.bt_KalmanAttTest);
        bt_posKF = (Button) findViewById(R.id.bt_KalmanPosTest);
        bt_motorTest = (Button) findViewById(R.id.bt_MotorsTest);
        bt_commTest = (Button) findViewById(R.id.bt_CommTest);


        bt_attKF.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick (View view){
                Intent intentAttKF = new Intent(TestsActivity.this, AttitudeKFTest.class);
                startActivity(intentAttKF);
                finish();

            }

        });

        bt_posKF.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick (View view){
                Intent intentPosKF = new Intent(TestsActivity.this, PositionKFTest.class);
                startActivity(intentPosKF);
                finish();

            }

        });

        bt_motorTest.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick (View view){
                Intent intentMotor = new Intent(TestsActivity.this, MotorsTestActivity.class);
                startActivity(intentMotor);
                finish();

            }

        });

        bt_commTest.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick (View view){
                Intent intentComm = new Intent(TestsActivity.this, TcpTest.class);
                startActivity(intentComm);
                finish();

            }

        });

    }

    @Override
    public void onBackPressed() {
        finish();
        Intent intentMainMenu = new Intent(TestsActivity.this, MainActivity.class);
        startActivity(intentMainMenu);
        return;
    }
}
