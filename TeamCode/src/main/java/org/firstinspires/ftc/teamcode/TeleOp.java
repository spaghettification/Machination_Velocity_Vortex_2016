package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 10/2/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop", group = "6994 Bot")
public class TeleOp extends FTC_6994_Template {
    public final PathSeg[] GyroTest = {new PathSeg(.25, 0, 24, DriveStyle.Linear)};
    enum Color{Red,Blue,Null}
    Color beaconColorSensorReturn;


    @Override
    public void init() {
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CapBallLiftLeft=hardwareMap.dcMotor.get(capBallLiftLeft);
        CapBallLiftRight=hardwareMap.dcMotor.get(capBallLiftRight);
        CapBallLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        CapBallFork=hardwareMap.servo.get(capBallFork);
        CapBallarm1=hardwareMap.crservo.get(capBallarm1);
        CapBallarm2=hardwareMap.crservo.get(capBallarm2);
        BallControl=hardwareMap.servo.get(ballControll);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        Gyro = hardwareMap.gyroSensor.get(gyroSensor);
        Catapult = hardwareMap.dcMotor.get(catapult);
        CatapultStop = hardwareMap.touchSensor.get(catapultStop);
        ButtonPusherLeft = hardwareMap.servo.get(buttonPusherLeft);
        ButtonPusherRight = hardwareMap.servo.get(buttonPusherRight);
        ButtonPusherLeft.setPosition(buttonPusherLeftStartPositoin);
        ButtonPusherRight.setPosition(buttonPusherRightStartPositoin);
        WhiteLineFinder=hardwareMap.colorSensor.get(whiteLineFinder);
        SideRangeSensor= hardwareMap.get(ModernRoboticsI2cRangeSensor.class,sideRangeSensor);
        FrontRangeSensor= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, frontRangeSensor);
        BackRangeSensor= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, backRangeSensor);
        BeaconCS=hardwareMap.colorSensor.get(beaconColorSensor);
        beaconColorSensorReturn = null;


    }

    public void start() {
        mRuntime.reset();
    }

    @Override
    public void loop() {
        BallCollection.setPower(-scaleInput(gamepad2.right_stick_y));
            if (gamepad1.left_bumper){
                FrontLeft.setPower(scaleInput(gamepad1.left_stick_y/2));
                FrontRight.setPower(scaleInput(-gamepad1.right_stick_y/2));
                BackLeft.setPower(scaleInput(gamepad1.left_stick_y/2));
                BackRight.setPower(scaleInput(-gamepad1.right_stick_y/2));}
            else{
                FrontLeft.setPower(scaleInput(gamepad1.left_stick_y));
                FrontRight.setPower(scaleInput(-gamepad1.right_stick_y));
                BackLeft.setPower(scaleInput(gamepad1.left_stick_y));
                BackRight.setPower(scaleInput(-gamepad1.right_stick_y));}

            if (CatapultStop.isPressed()){
                if (gamepad2.dpad_up){Catapult.setPower(1);}
                else {Catapult.setPower(0);}}

            else{
                if (gamepad2.dpad_down){Catapult.setPower(1);}
            else {Catapult.setPower(0);}
            }
            //Catapult.setPower(Range.clip(gamepad2.left_stick_x,0,1));
            //CapBallLiftRight.setPower(Range.clip(scaleInput(gamepad2.left_stick_y),-1,1)*.8);
            //CapBallLiftLeft.setPower(Range.clip(scaleInput(gamepad2.left_stick_y),-1,1));
            /*if (gamepad2.a){CapBallarm1.setPower(1);}
                else if (gamepad2.b){CapBallarm1.setPower(1); CapBallarm1.setDirection(DcMotorSimple.Direction.REVERSE);}
                    else CapBallarm1.setPower(.5);

            if (gamepad2.x){CapBallarm2.setPower(1);}
                else if (gamepad2.y){CapBallarm2.setPower(1); CapBallarm2.setDirection(DcMotorSimple.Direction.REVERSE);}
                    else CapBallarm2.setPower(.5);*/

            if (gamepad1.left_bumper){BallControl.setPosition(ballControlEngagedPosition);}

            if (gamepad1.right_bumper){BallControl.setPosition(ballControlStartPosition);}

            /*if (gamepad1.left_bumper){ButtonPusherLeft.setPosition(buttonPusherLeftEngagedPositoin);}
                else ButtonPusherLeft.setPosition(buttonPusherLeftStartPositoin);

            if (gamepad1.right_bumper){ButtonPusherRight.setPosition(buttonPusherRightEngagedPositoin);}
                else ButtonPusherRight.setPosition(buttonPusherRightStartPositoin);*/

            if (BeaconCS.blue()>BeaconCS.red()&&BeaconCS.blue()>BeaconCS.green()){
                 beaconColorSensorReturn=Color.Blue;
                }
            else if (BeaconCS.red()>BeaconCS.blue()&&BeaconCS.red()>BeaconCS.green()){
                 beaconColorSensorReturn=Color.Red;
                }
            else {beaconColorSensorReturn=Color.Null;}
        telemetry.addData("Red", WhiteLineFinder.red());
        telemetry.addData("Blue", WhiteLineFinder.blue());
        telemetry.addData("Green", WhiteLineFinder.green());
        telemetry.addData("BackRightSide Range",SideRangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("BackRight Range",FrontRangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("BackLeft Range",BackRangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Ball Control Position", BallControl.getPosition());
        telemetry.addData("Catapult Primed", CatapultStop.isPressed());
        telemetry.addData("Gyro", getIntegratedZValue());
        telemetry.addData("ThetaPower Consstant", ThetaPowerConstant);
        telemetry.addData("FrontLeft Power", FrontLeft.getPower());
        telemetry.addData("FrontRight Power", FrontRight.getPower());
        telemetry.addData("BackLeft Power", BackLeft.getPower());
        telemetry.addData("BackRight Power", BackRight.getPower());
        telemetry.addData("Catapult Power", Catapult.getPower());
        telemetry.addData("Ball Collector Power", BallCollection.getPower());
        telemetry.addData("Left Button Pusher Positoin", ButtonPusherLeft.getPosition());
        telemetry.addData("RightButton Pusher Positoin", ButtonPusherRight.getPosition());
    }

    public void stop() {
        setDrivePower(0, 0, 0, 0);
    }

}

    /*public void primecatapult(DcMotor motor,*//* Servo servo,*//* TouchSensor sensor, boolean ready) {
        boolean CatapultStopPrime = CatapultStop.isPressed();
        telemetry.addData("CatapultStat", Status);
*//*
        telemetry.addData("i", i);
        telemetry.addData("j", j);
*//*
        telemetry.addData("Touch sensor pressed", sensor.isPressed());
        telemetry.addData("Gamepad button A pressed", gamepad1.a);
*//*
        Status = CatapultStatus.Prime;
        switch (Status) {
            case Prime: {
                if (!CatapultStopPrime) {
                    motor.setPower(1);
                } else {
                    motor.setPower(0);
                    Status = CatapultStatus.Launch;
                }
            }
            break;
            case Launch: {
                if (CatapultStopPrime) {
                    if (ready) {
                        motor.setPower(1);
                    } else {
                        motor.setPower(0);
                    }
                } else {
                    Status = CatapultStatus.Prime;
                }

            }
            break;
        }*//*




        *//*switch (h) {
            case 1: {
                switch (i) {
                    case 1: {
                        if (!sensor.isPressed()) {
                            motor.setPower(1);
                        } else {
                            motor.setPower(0);
                            i++;
                        }
                    }
                    break;
                    case 2: {
                        h++;
                        j = 1;
                    }
                    break;
                }
            }
            break;
            case 2: {
                switch (j) {
                    case 1: {
                        motor.setPower(Range.clip(gamepad1.right_stick_y, 0, 1));
                        if (!sensor.isPressed()) {
                            j++;
                        }

                    }
                    break;
                    case 2: {
                        h--;
                        i = 1;

                    }
                    break;
                }
                if (gamepad1.a) {

                    i--;
                }
                *//**//*switch (j){

                    case 1:{motor.setPower(.5


                    );
                    }break;
                    case 2:{
                        i=1;
                        Status=CatapultStatus.Prime;
                    }break;
                }*//**//*
            }
            default: {
                Status = CatapultStatus.Prime;
            }
            //break;
        }*//*/*

*//*
    }

    public enum CatapultStatus {Prime, Launch}
}
*/