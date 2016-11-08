package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Trevor on 10/29/2016.
 */
public class LinearTeleop extends OpMode {
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Catapult;
    DcMotor BallCollection;
    DcMotor CapBallLiftLeft;
    DcMotor CapBallLiftRight;
    Servo CapBallGrabber;
    Servo CapBallSwivel;
    Servo CapBallBlock;
    Servo BallControl;
    Servo ButtonPusherLeft;
    Servo ButtonPusherRight;
    Servo servo8;
    Servo servo9;
    Servo servo10;
    Servo servo11;
    Servo servo12;
    ModernRoboticsI2cRangeSensor TopRightRangeSensor;
    ModernRoboticsI2cRangeSensor TopLeftRangeSensor;
    ModernRoboticsI2cRangeSensor BottomRightRangeSensor;
    ModernRoboticsI2cRangeSensor BottomLeftRangeSensor;
    TouchSensor CatapultStop;
    GyroSensor Gyro;

    public enum CatapultStatus {Prime, Launch}
    CatapultStatus Status;

    public void init(){
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BallCollection = hardwareMap.dcMotor.get("BC");
        Gyro = hardwareMap.gyroSensor.get("G");
        Catapult = hardwareMap.dcMotor.get("C");
        CatapultStop = hardwareMap.touchSensor.get("CS");
        ButtonPusherLeft = hardwareMap.servo.get("BPL");
        ButtonPusherRight = hardwareMap.servo.get("BPR");
        ButtonPusherLeft.setPosition(1);
        ButtonPusherRight.setPosition(0);
    }
        public void loop(){
            FrontLeft.setPower(scaleInput(Range.clip(gamepad1.left_stick_y,-1,1)));
            FrontRight.setPower(scaleInput(Range.clip(gamepad1.right_stick_y,-1,1)));
            BackLeft.setPower(scaleInput(Range.clip(gamepad1.left_stick_y,-1,1)));
            BackRight.setPower(scaleInput(Range.clip(gamepad1.right_stick_y,-1,1)));

            int i=0;
            boolean activationButton = gamepad1.a;
            boolean Catapultstop = CatapultStop.isPressed();
            switch(i){
                case 0:{
                    if (Catapultstop=false){
                        while (Catapultstop=false){
                            Catapult.setPower(1);
                        }Catapult.setPower(0);i++;
                    }
                }break;
                case 1:{
                    if (activationButton){
                        while(Catapultstop=true)
                        {Catapult.setPower(1);}
                        i--;
                    }
                }break;
            }

            boolean buttonB=gamepad1.b;
            if (buttonB=true){BallCollection.setPower(1);}
            else {BallCollection.setPower(0);}

        }

    public double scaleInput(double dVal) {
        //first few numbers are zero to account for offest of joystick keeping motors at zero power until acted upon by driver
        double[] scaleArray = {0.0, 0.0, 0.2, 0.22, 0.28, 0.35, 0.40,
                0.45, 0.50, 0.55, 0.60, 0.65, 0.72, 0.85, .90, 1.0, 1.0
        };
        int index = (int) (dVal * 15.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 15) {
            index = 15;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        if (dVal < -1) {
            dVal = -1;
        }

        if (dVal > 1) {
            dVal = 1;
        }


        return dScale;
    }

}
