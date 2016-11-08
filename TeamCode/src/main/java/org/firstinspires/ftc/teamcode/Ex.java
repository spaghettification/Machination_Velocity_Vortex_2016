package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Trevor on 10/23/2016.
 */
public class Ex extends OpMode {
    DcMotor Left;
    DcMotor Right;
    enum State{Path1,Path2}
    int PathSeg;
    GyroSensor Gyro;
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
    public void Path(State state){
        PathSeg=0;

        switch (state){
            case Path1:{
                switch (PathSeg){
                    case 0:{PathSeg++;}break;
                    case 1:{PathSeg++;}break;
                }
            }break;
            case Path2:{
                switch (PathSeg){
                    case 0:{PathSeg++;}break;
                    case 1:{PathSeg++;}break;
            }break;
        }
    }
}
    public void DriveWithEncoder(int Distance,double Power){
        while (Math.abs(Left.getCurrentPosition()-Distance)>0&&(Right.getCurrentPosition()-Distance)>0){
            Left.setPower(Power);
            Right.setPower(Power);
        }
        Left.setPower(0);
        Right.setPower(0);
    }
    public void DriveUsingGyro(int TargetAngle,double Power){
        while ((Gyro.getHeading()-TargetAngle)>1){
            Left.setPower(-Power);
            Right.setPower(Power);}
        Left.setPower(0);
        Right.setPower(0);
    }
}
