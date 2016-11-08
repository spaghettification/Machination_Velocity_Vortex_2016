package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Trevor on 10/25/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "6994 Bot")
public class test extends OpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    @Override
    public void init() {
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTargetPositionOfDriveMotors(0,0,0,0);
    }
    int i=0;

    @Override
    public void loop() {
        switch (i){
            case 0:{
                    setDriveMotorPower(-.5);
                    setTargetPositionOfDriveMotors(-28,-28,-28,-28);
                    setDriveMaxSpeed(3000);

                    setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if ((FrontLeft.getCurrentPosition()>FrontLeft.getTargetPosition()&&FrontRight.getCurrentPosition()>FrontRight.getTargetPosition()&&BackLeft.getCurrentPosition()>BackLeft.getTargetPosition()&&BackRight.getCurrentPosition()>BackRight.getTargetPosition())){
                    i++;}
            }break;
            case 1:{
                setDriveMotorPower(.5);
                setTargetPositionOfDriveMotors(28,28,28,28);
                setDriveMaxSpeed(3000);
                setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                if ((FrontLeft.getCurrentPosition()>FrontLeft.getTargetPosition()&&FrontRight.getCurrentPosition()>FrontRight.getTargetPosition()&&BackLeft.getCurrentPosition()>BackLeft.getTargetPosition()&&BackRight.getCurrentPosition()>BackRight.getTargetPosition())){
                }
            }break;
            case 2:{
                setTargetPositionOfDriveMotors(28,28,28,28);
                setDriveMaxSpeed(5000);
                setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                if ((FrontLeft.getCurrentPosition()>FrontLeft.getTargetPosition()&&FrontRight.getCurrentPosition()>FrontRight.getTargetPosition()&&BackLeft.getCurrentPosition()>BackLeft.getTargetPosition()&&BackRight.getCurrentPosition()>BackRight.getTargetPosition())){
                }
            }break;
            case 3:{
                setTargetPositionOfDriveMotors(-28,-28,-28,-28);
                setDriveMaxSpeed(5000);
                setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                if ((FrontLeft.getCurrentPosition()>FrontLeft.getTargetPosition()&&FrontRight.getCurrentPosition()>FrontRight.getTargetPosition()&&BackLeft.getCurrentPosition()>BackLeft.getTargetPosition()&&BackRight.getCurrentPosition()>BackRight.getTargetPosition())){
                }}break;}
            telemetry.addData("FrontLeft",FrontLeft.getTargetPosition());
            telemetry.addData("FrontRight",FrontRight.getTargetPosition());
            telemetry.addData("BackLeft",BackLeft.getTargetPosition());
            telemetry.addData("BackRight",BackRight.getTargetPosition());

            telemetry.addData("FrontLeft",FrontLeft.getCurrentPosition());
            telemetry.addData("FrontRight",FrontRight.getCurrentPosition());
            telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
            telemetry.addData("BackRight",BackRight.getCurrentPosition());


    }
    public void setDriveMotorMode(DcMotor.RunMode Mode) {
        FrontLeft.setMode(Mode);
        FrontRight.setMode(Mode);
        BackLeft.setMode(Mode);
        BackRight.setMode(Mode);
    }

    double encoderTicksPerInch = 87.19;

    public void setTargetPositionOfDriveMotors(int FLDistanceInInches, int FRDistanceInInches, int BLDistanceInInches, int BRDistanceInInches) {
        FrontLeft.setTargetPosition((int) (FLDistanceInInches * encoderTicksPerInch));
        FrontRight.setTargetPosition((int) (FRDistanceInInches * encoderTicksPerInch));
        BackLeft.setTargetPosition((int) (BLDistanceInInches * encoderTicksPerInch));
        BackRight.setTargetPosition((int) (BRDistanceInInches * encoderTicksPerInch));
    }

    public void setDriveMotorPower(double Power) {
        FrontLeft.setPower(Power);
        FrontRight.setPower(Power);
        BackLeft.setPower(Power);
        BackRight.setPower(Power);
    }

    public void setDriveMaxSpeed(int EncoderTicksPerSecond) {
        FrontLeft.setMaxSpeed(EncoderTicksPerSecond);
        FrontRight.setMaxSpeed(EncoderTicksPerSecond);
        BackLeft.setMaxSpeed(EncoderTicksPerSecond);
        BackRight.setMaxSpeed(EncoderTicksPerSecond);
    }
}
