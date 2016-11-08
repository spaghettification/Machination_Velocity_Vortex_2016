package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Trevor on 10/10/2016.
 */
public class Examples extends OpMode {

    int WheelDiameter = 4;
    double EncoderTickToInch = 1440 * WheelDiameter * Math.PI;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    public enum Direction {Forward, Backwards, TurnLeft, TurnRight}
    @Override
    public void init() {
    initializeEncoders();
        //Dont Forget to initialize your motors
    }

    @Override
    public void loop() {
        if(!MotorsBusy()){
            Drive(.5,Direction.Forward,12);
        }

    }

    public void initializeEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public boolean MotorsBusy() {
        return FrontLeft.isBusy()&&FrontRight.isBusy()&&BackLeft.isBusy()&&BackRight.isBusy();
    }

    public void Drive(double power, Direction direction, int Distance) {
        int DistanceInInches= (int) (Distance*EncoderTickToInch);
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);

        switch (direction) {
            case Forward: {
                FrontLeft.setTargetPosition(DistanceInInches);
                FrontRight.setTargetPosition(DistanceInInches);
                BackLeft.setTargetPosition(DistanceInInches);
                BackRight.setTargetPosition(DistanceInInches);
            }
            break;
            case Backwards: {
                FrontLeft.setTargetPosition(-DistanceInInches);
                FrontRight.setTargetPosition(-DistanceInInches);
                BackLeft.setTargetPosition(-DistanceInInches);
                BackRight.setTargetPosition(-DistanceInInches);
            }
            break;
            case TurnLeft: {
                FrontLeft.setTargetPosition(-DistanceInInches);
                FrontRight.setTargetPosition(DistanceInInches);
                BackLeft.setTargetPosition(-DistanceInInches);
                BackRight.setTargetPosition(DistanceInInches);
            }
            break;
            case TurnRight: {
                FrontLeft.setTargetPosition(DistanceInInches);
                FrontRight.setTargetPosition(-DistanceInInches);
                BackLeft.setTargetPosition(DistanceInInches);
                BackRight.setTargetPosition(-DistanceInInches);
            }
            break;

        }

    }


}
