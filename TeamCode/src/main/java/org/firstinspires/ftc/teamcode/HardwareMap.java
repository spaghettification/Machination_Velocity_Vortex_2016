package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Trevor on 11/5/2016.
 */
public abstract class HardwareMap extends OpMode {
    DcMotor                                         FrontLeft;
    DcMotor                                         FrontRight;
    DcMotor                                         BackLeft;
    DcMotor                                         BackRight;
    DcMotor                                         Catapult;
    DcMotor                                         BallCollection;
    DcMotor                                         CapBallLiftLeft;
    DcMotor                                         CapBallLiftRight;

    GyroSensor                                      Gyro;

    ColorSensor                                     BeaconColorSensor;
    ColorSensor                                     WhiteLineFinder;

    ModernRoboticsI2cRangeSensor                    BackRightSideRangeSensor;
    ModernRoboticsI2cRangeSensor                    BackRightRangeSensor;
    ModernRoboticsI2cRangeSensor                    BackLeftRangeSensor;

    TouchSensor CatapultStop;

    Servo                                           ButtonPusherLeft;
    Servo                                           ButtonPusherRight;
    Servo                                           CapBallFork;

    Servo                                           servo6;
    Servo                                           servo7;
    Servo                                           servo8;
    Servo                                           servo9;
    Servo                                           servo10;
    Servo                                           servo11;
    Servo                                           servo12;

    CRServo                                         CapBallarm1;
    CRServo                                         CapBallarm2;
    Servo                                           BallControl;


    public String frontLeftMotor                    = "fl";
    public String frontRightMotor                   = "fr";
    public String backLeftMotor                     = "bl";
    public String backRightMotor                    = "br";
    public String catapult                          = "c";
    public String ballCollection                    = "bc";
    public String capBallLiftLeft                   = "cbll";
    public String capBallLiftRight                  = "cblr";
    public String gyroSensor                        = "gyro";
    public String beaconColorSensor                 = "bcs";
    public String whiteLineFinder                   = "wlf";
    public String backRightSideRangeSensor          = "brsrs";
    public String backRightRangeSensor              = "brrs";
    public String backLeftRangeSensor               = "blrs";
    public String catapultStop                      = "cs";
    public String capBallFork                       = "cbf";
    public String capBallarm1                       = "cba1";
    public String capBallarm2                       = "cba2";
    public String ballControll                      = "bc";
    public String buttonPusherLeft                  = "bpl";
    public String buttonPusherRight                 = "bpr";

    public double buttonPusherLeftStartPositoin     = 0;
    public double buttonPusherRightStartPositoin    = 1;
    public double buttonPusherLeftEngagedPositoin   = 0;
    public double buttonPusherRightEngagedPositoin  = 1;
    public double ballControlStartPosition          = 0;
    public double ballControlEngagedPosition        = 1;

    public float LinearproportionalConstant         =0;
    public float LinearintegralConstant             =0;
    public float LinearderivitiveConstant           =0;
    public float AngularproportionalConstant        =0;
    public float AngularintegralConstant            =0;
    public float AngularderivitiveConstant          =0;

    public double AndyMarkMotor_TicksPerRevolution  =1120;
    public double CountsPerInch=(AndyMarkMotor_TicksPerRevolution/4*Math.PI);


    public float AngularMaxCorrection               =100;
    public float AngularMinCorrection               =15;
    public float LinearMaxCorrection                =100;
    public float LinearMinCorrection                =15;

    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    public enum DriveSyle{LinearWithEnocders, ClockwiseWithGyroScope,CounterClockwiseWithGyroScope}




}
