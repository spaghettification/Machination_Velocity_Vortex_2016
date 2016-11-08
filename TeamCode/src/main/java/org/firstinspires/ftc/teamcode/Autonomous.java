package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

/**
 * Created by Trevor on 10/23/2016.
 */

//Does not work
public class Autonomous extends HardwareMap {
    int FrontLeftEncTarget;
    int FrontRightEncTarget;
    int BackLeftEncTarget;
    int BackRightEncTarget;

    public void init(){

        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);


        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        state=State.A;

    }
    public enum State{A,B,C}
    State state;
    @Override
    public void init_loop() {


    }
    public void loop(){

        if (!motorsAreBusy()){


            setTargetPosition(2880,2880,2880,2880);
            setDriveMotorPower(.125,.125,.125,.125);

        }


        telemetry.addData("FrontLeft Power", FrontLeft.getPower());
        telemetry.addData("FrontRight Power", FrontRight.getPower());
        telemetry.addData("BackLeft Power", BackLeft.getPower());
        telemetry.addData("BackRight Power", BackRight.getPower());

        telemetry.addData("FrontLeft Encoder", FrontLeft.getCurrentPosition());
        telemetry.addData("FrontRight Encoder", FrontRight.getCurrentPosition());
        telemetry.addData("BackLeft Encoder", BackLeft.getCurrentPosition());
        telemetry.addData("BackRight Encoder", BackRight.getCurrentPosition());


        telemetry.addData("FrontLeft Target", FrontLeft.getTargetPosition());
        telemetry.addData("FrontRight Target", FrontRight.getTargetPosition());
        telemetry.addData("BackLeft Target", BackLeft.getTargetPosition());
        telemetry.addData("BackRight Target", BackRight.getTargetPosition());
    }
    public void setDriveMotorPower(double FrontLeftPower,double FrontRightPower,double BackLeftPower,double BackRightPower){
        FrontLeft.setPower(FrontLeftPower);
        FrontRight.setPower(FrontRightPower);
        BackLeft.setPower(BackLeftPower);
        BackRight.setPower(BackRightPower);
    }
    public void setDriveMotorMode(DcMotor.RunMode Mode){
        FrontLeft.setMode(Mode);
        FrontRight.setMode(Mode);
        BackLeft.setMode(Mode);
        BackRight.setMode(Mode);
    }
    public void setTargetPosition(int FrontLeftEncoderTarget,int FrontRightEncoderTarget,int BackLeftEncoderTarget,int BackRightEncoderTarget){
        FrontLeft.setTargetPosition(FrontLeftEncTarget=FrontLeftEncoderTarget);
        FrontRight.setTargetPosition(FrontRightEncTarget=FrontRightEncoderTarget);
        BackLeft.setTargetPosition(BackLeftEncTarget=BackLeftEncoderTarget);
        BackRight.setTargetPosition(BackRightEncTarget = BackRightEncoderTarget);
    }
    public void addEncoderTarget(int FrontLeftEncoderTarget,int FrontRightEncoderTarget,int BackLeftEncoderTarget,int BackRightEncoderTarget){
        FrontLeft.setTargetPosition(FrontLeftEncTarget+=FrontLeftEncoderTarget);
        FrontRight.setTargetPosition(FrontRightEncTarget+=FrontRightEncoderTarget);
        BackLeft.setTargetPosition(BackLeftEncTarget+=BackLeftEncoderTarget);
        BackRight.setTargetPosition(BackRightEncTarget += BackRightEncoderTarget);
    }
    public boolean motorsAreBusy(){return FrontLeft.isBusy()&&FrontRight.isBusy()&&BackLeft.isBusy()&&BackRight.isBusy();}

    public boolean encoderTargetReached(){
        return (FrontLeftEncTarget<FrontLeft.getCurrentPosition()&&FrontRightEncTarget<FrontRight.getCurrentPosition()&&BackRightEncTarget<BackLeft.getCurrentPosition()&&BackRightEncTarget<BackRight.getCurrentPosition());
    }
}

