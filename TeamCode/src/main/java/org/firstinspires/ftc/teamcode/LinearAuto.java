package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 10/23/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LinearAuto", group = "6994 Bot")
public class LinearAuto extends LinearOpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    enum turnDirection {Clockwise, CounterClockwise}

    ModernRoboticsI2cRangeSensor RightSideRange;
    ModernRoboticsI2cRangeSensor BackRightRange;
    ModernRoboticsI2cRangeSensor BackLeftRange;
    GyroSensor Gyro;
    OpticalDistanceSensor ODS;
    double LightDectectedAtStart = 0;
    double CountsPerInch = 89.17;

    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        Gyro = hardwareMap.gyroSensor.get("G");
        RightSideRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "RSS");
        BackRightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BRS");
        BackLeftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "BLS");
        ODS = hardwareMap.opticalDistanceSensor.get("ods");
        waitForStart();
        LightDectectedAtStart = ODS.getLightDetected();
        Thread.sleep(200);
        path(Path.ToSecondBeacon, .3);


    }

    enum Path {ToFirstBeacon, ToSecondBeacon};
    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.

        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    public void path(Path path, double Power) {
        int PathSegement = 0;
        double distanceFromSideWall = 0;
        double rightSideDistanceFromBackWall = 0;
        double leftSideDistanceFromBackWall = 0;
        double distanceFromBackWall = 0;
        double thetaOfPath;
        double hypotenuseOfPath;
        int XBuffer = 6;
        int YBuffer = 3;
        double distanceToTape = 36;
        switch (path) {
            case ToFirstBeacon: {
                switch (PathSegement) {
                    case 0: {//SquareRobotWithWall

                        while (BackRightRange.getDistance(DistanceUnit.INCH) != BackLeftRange.getDistance(DistanceUnit.INCH)) {
                            if ((BackRightRange.getDistance(DistanceUnit.INCH) > BackLeftRange.getDistance(DistanceUnit.INCH))) {
                                setDrivePower(0, -.0625, 0, -.0625);
                            } else setDrivePower(-.0625, 0, -.0625, 0);
                        }
                        if (BackRightRange.getDistance(DistanceUnit.INCH) == BackLeftRange.getDistance(DistanceUnit.INCH)) {

                            PathSegement++;
                        }
                    }
                    break;


                    case 1: {
                        distanceFromSideWall = RightSideRange.getDistance(DistanceUnit.INCH);
                        rightSideDistanceFromBackWall = BackRightRange.getDistance(DistanceUnit.INCH);
                        leftSideDistanceFromBackWall = BackLeftRange.getDistance(DistanceUnit.INCH);
                        distanceFromBackWall = (rightSideDistanceFromBackWall + leftSideDistanceFromBackWall) / 2;


                        thetaOfPath = 90 - Math.atan((distanceToTape - distanceFromBackWall) / (distanceFromSideWall - XBuffer));
                        while (1 < Math.abs(getIntegratedZValue() - thetaOfPath)) {
                            setDrivePower(.08, -.08, .08, -.08);
                        }
                        if ((1 < Math.abs(getIntegratedZValue() - thetaOfPath))) {
                            PathSegement++;
                        }


                    }
                    break;
                    case 2: {
                        hypotenuseOfPath = Math.sqrt(((distanceFromSideWall - XBuffer) * (distanceFromSideWall - XBuffer) + ((distanceToTape - distanceFromBackWall) * (distanceToTape - distanceFromBackWall))));
                        while (!Travelcomplete((int) hypotenuseOfPath)) {
                            setDrivePower(Power, Power, Power, Power);
                        }
                        if (Travelcomplete((int) hypotenuseOfPath)) {
                            setDrivePower(0, 0, 0, 0);
                            PathSegement++;
                        }
                    }
                    break;
                    case 3: {
                        while (getIntegratedZValue() > 0) {
                            setDrivePower(-.0625, 0, 0, 0);
                        }
                        setDrivePower(0, 0, 0, 0);
                        PathSegement++;
                    }
                    break;

                }
            }
            break;
            case ToSecondBeacon: {
                switch (PathSegement) {
                    case 0: {
                        int Distance = 24;
                        while (!Travelcomplete(Distance)) {
                            setDrivePower(Power, Power, Power, Power);
                        }
                        if (Travelcomplete(Distance)) {
                            setDrivePower(0, 0, 0, 0);
                            PathSegement++;
                        }
                    }
                    break;
                    case 2: {
                        while(!Travelcomplete(12)){
                            setDrivePower(-Power,-Power,Power,-Power);
                        }
                        if (Travelcomplete(12)){setDrivePower(0,0,0,0);}
                    }
                    break;
                    case 1: {
                        int Distance = 24;
                        while (!Travelcomplete(Distance)) {
                            setDrivePower(Power, Power, Power, Power);
                        }
                        if (Travelcomplete(Distance)) {
                            setDrivePower(0, 0, 0, 0);
                            PathSegement++;
                        }
                    }
                    break;
                }
            }
            break;
        }
    }

    public void setDrivePower(double FLPower, double FRPower, double BLPower, double BRPower) {
        FrontLeft.setPower(FLPower);
        FrontRight.setPower(FRPower);
        FrontLeft.setPower(BLPower);
        FrontLeft.setPower(BRPower);
    }

    public boolean Travelcomplete(int Distance) {
        /*int FrontLeftEncoderPosition=FrontLeft.getCurrentPosition();
        int FrontRightEncoderPosition=FrontRight.getCurrentPosition();
        int BackLeftEncoderPosition=BackLeft.getCurrentPosition();
        int BackRightEncoderPosition=BackRight.getCurrentPosition();
        return (Math.abs(FrontLeft.getCurrentPosition()-(FrontLeftEncoderPosition+Distance))<5&&(Math.abs(FrontRight.getCurrentPosition()-(FrontRightEncoderPosition+Distance))<5&&(Math.abs(BackLeft.getCurrentPosition()-(BackLeftEncoderPosition+Distance))<5&&(Math.abs(BackRight.getCurrentPosition()-(BackRightEncoderPosition+Distance))<5))));*/
        return ((Math.abs(FrontLeft.getTargetPosition() - FrontLeft.getCurrentPosition())) < 5 && (Math.abs(FrontRight.getTargetPosition() - FrontRight.getCurrentPosition())) < 5 && (Math.abs(BackLeft.getTargetPosition() - BackLeft.getCurrentPosition()) < 5) && (Math.abs(BackRight.getTargetPosition() - BackRight.getCurrentPosition()) < 5));
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

    public boolean gyroAtTaretAngle(int TargetAngle) {
        return (Math.abs(getIntegratedZValue() - TargetAngle) > .75);
    }

    public boolean WhiteLineFound() {
        return (ODS.getLightDetected() - LightDectectedAtStart > 10);
    }

    enum DriveStyle {Linear, ClockWise, CounterClockwise, FindWhiteLine}

    public void DriveUsingEncoders(DriveStyle driveStyle, int Distance, int Speed, double Power, int TargetAngle, double BaseLineODS) {/*
        if (!FrontLeft.isBusy()&&!FrontRight.isBusy()&&!BackLeft.isBusy()&&!BackRight.isBusy()){*/
        switch (driveStyle) {
            case Linear: {
                setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setTargetPositionOfDriveMotors(Distance, Distance, Distance, Distance);
                setDriveMaxSpeed(Speed);
                setDriveMotorPower(Power);

            }
            break;
            case ClockWise: {
                while (!gyroAtTaretAngle(TargetAngle)){
                    setDriveMotorPower(Power);
                    setTargetPositionOfDriveMotors(100000,-100000,100000,-100000);
                }

            }
            break;
            case CounterClockwise: {
            }
            break;
            case FindWhiteLine: {
            }
            break;

        }
    }
}
