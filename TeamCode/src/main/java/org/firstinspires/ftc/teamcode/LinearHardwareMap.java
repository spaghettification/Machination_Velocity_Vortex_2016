package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.sql.Ref;
import java.util.Set;

/**
 * Created by Trevor on 11/5/2016.
 */
public abstract class LinearHardwareMap extends LinearOpMode {
    public DcMotor                                              FrontLeft;
    public DcMotor                                              FrontRight;
    public DcMotor                                              BackLeft;
    public DcMotor                                              BackRight;
    public DcMotor                                              Catapult;
    public DcMotor                                              BallCollection;
    public DcMotor                                              CapBallLiftLeft;
    public DcMotor                                              ButtonPusherActuator;
    public GyroSensor                                           Gyro;
    public ColorSensor                                          BeaconColorSensor;
    public OpticalDistanceSensor                                LeftWhiteLineFinder;
    public OpticalDistanceSensor                                RightWhiteLineFinder;
    public ModernRoboticsI2cRangeSensor                         SideRangeSensor;
    public ModernRoboticsI2cRangeSensor                         FrontRangeSensor;
    public ModernRoboticsI2cRangeSensor                         BackRangeSensor;
    public TouchSensor                                          CatapultStop;
    public TouchSensor                                          ButtonPusherMax;
    public TouchSensor                                          ButtonPusherMin;
    public Servo                                                ButtonPusherArm;
    public Servo                                                ButtonPusher;
    public Servo                                                CapBallFork;
    public Servo                                                servo6;
    public Servo                                                servo7;
    public Servo                                                servo8;
    public Servo                                                servo9;
    public Servo                                                servo10;
    public Servo                                                servo11;
    public Servo                                                servo12;
    public CRServo                                              CapBallarm1;
    public CRServo                                              CapBallarm2;
    public Servo                                                BallControl;
    public DeviceInterfaceModule                                DIM;

    public String frontLeftMotor                                =      "fl"; //VTOJ Port 1
    public String frontRightMotor                               =      "fr"; // VTOJ Port 2
    public String backLeftMotor                                 =      "bl"; //VTOL Port 1
    public String backRightMotor                                =      "br";//VTOL Port 2
    public String catapult                                      =      "c"; // VTAV Port 1
    public String ballCollection                                =      "bc"; //VTAV Port 2
    public String capBallLiftLeft                               =      "cbll"; //SXSX
    public String buttonPusherActuator                          =      "bpa";   //SXSX
    public String gyroSensor                                    =      "gyro";
    public String beaconColorSensor                             =      "bcs";
    public String LeftwhiteLineFinder                           =      "Lwlf";
    public String RightwhiteLineFinder                          =      "Rwlf";
    public String sideRangeSensor                               =      "brsrs";
    public String frontRangeSensor                              =      "brrs";
    public String backRangeSensor                               =      "blrs";
    public String catapultStop                                  =      "cs";
    public String capBallFork                                   =      "cbf";
    public String capBallarm1                                   =      "cba1";
    public String capBallarm2                                   =      "cba2";
    public String ballControll                                  =      "ballco";
    public String buttonPusher                                  =      "bp";
    public String buttonPusherArm                               =      "bpa";
    public double ballControlStartPosition                      =      .7;
    public double ballControlEngagedPosition                    =       0;
    public double buttonPusherLeft                              =       1;
    public double buttonPusherRight                             =      .2;
    public double buttonPusherCenter                            =      .6;
    public float LinearproportionalConstant                     =       0;
    public float LinearintegralConstant                         =       0;
    public float LinearderivitiveConstant                       =       0;
    public float AngularproportionalConstant                    =       0;
    public float AngularintegralConstant                        =       0;
    public float AngularderivitiveConstant                      =       0;
    public double AndyMarkMotor_TicksPerRevolution              =       1120;
    public double CountsPerInch                                 =       89;
    public float AngularMaxCorrection                           =       100;
    public float AngularMinCorrection                           =       15;
    public float LinearMaxCorrection                            =       100;
    public float LinearMinCorrection                            =       15;


    double TurningConstant=.0125;
    String VuforiaLicenseKey = "AbkJpf//////AAAAGfwmmKkkGUDwrRcXe4puyLQhZ3m1wmsmuJUw2GVDtb7tWinUTnSd+UmyGz5aylC8ShWX8ayvA9h2mDtWnM1s3yni7S/WtH8buZO7gUBz9FotxNPJGL8Di9VJSmOhzEoyHLivQpx/vPwoH0Aejcvr1lBt8b5yMEgegLQ+WbmwNmj25ciaaMFDhryp7CTOzZFswvIUdhZ84PBJJew94ewMFjrsGNqra+0beno8wvEH9XmHp2kj9lVT+u8EjZdSQuEowkS5Lw2bnmOCMfPk9/00KZ+xBfaa2LDB3IXuYR2FVdd6qORTWXA8N120mYbCx8x8U7R4JdZs/eAH279CtHqFyFPdQtj3qn3Of7Z3urbcezNu";
    float Linearlasterror=0;
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime Totalruntime = new ElapsedTime();

    public int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    public void StopAllMotors() {
        if (FrontLeft != null) {
            FrontLeft.setPower(0);
        }
        if (FrontRight != null) {
            FrontRight.setPower(0);
        }
        if (BackRight != null) {
            BackLeft.setPower(0);
        }
        if (BackRight != null) {
            BackRight.setPower(0);
        }
        if (Catapult != null) {
            Catapult.setPower(0);
        }
        if (BallCollection != null) {
            BallCollection.setPower(0);
        }
        if (ButtonPusherActuator != null) {
            ButtonPusherActuator.setPower(0);
        }
        if (CapBallLiftLeft != null) {
            CapBallLiftLeft.setPower(0);
        }
    }
    public void PrepareAutonomous(){
        AutonomousHardwareMap();
        StopAllMotors();
        InitializeServoPositions();
        Gyro.calibrate();
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while(Gyro.isCalibrating()){

        }
    }
    public void DriveToFirstBeacon(String Color){
        if (Color.toLowerCase()=="blue"){
        Turn(.25,0,25,true);
        DriveWithPID(.375,48,25,3000);
        Turn(0,.25,0,true);
        FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,.125,.6);}
        else{
            Turn(0,.25,-25,true);
            DriveWithPID(.375,-48,-25,3000);
            Turn(.25,0,0,true);
            FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,-.125,.6);}
    }
    public void DriveFromFirstBeaconToSecondBeacon(String Color){
        if (Color.toLowerCase()=="blue"){
        DriveWithPID(.375,40,0,2500);
        FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,.125,.6);}
        else{
            DriveWithPID(.375,-40,0,2500);
            FindWhiteLine(LeftWhiteLineFinder,RightWhiteLineFinder,-.125,.6);}
    }
    public void DriveFromSecondButtonToShootSpot(String Color){
        if (Color.toLowerCase()=="blue"){
        Turn(0,.25,-150,true);
        DriveWithPID(.375,24,-150,3000);}
        else{Turn(.25,0,150,true);
            DriveWithPID(.375,24,150,3000);}
    }

    public void DriveFromShootSpotToCapBallStand(String Color){
        if (Color.toLowerCase()=="blue"){
        DriveWithPID(.375,30,25,3000);
        }
        else{DriveWithPID(.375,-30,-25,3000);}
    }

    public double PidPowerAdjustment(int TargetAngle) {

        float LinearCumulativeerror = 0;
        float LinearproportionalCorrection;
        float LinearintegralCorrection;
        float LinearSlopeofderivitive;
        float LinearMaxCorrection = 100;
        float LinearMinCorrection = 15;
        float Linearerror = Math.abs(TargetAngle - getIntegratedZValue());
        LinearproportionalCorrection = (LinearproportionalConstant * Linearerror);
        LinearCumulativeerror += Linearerror;
        LinearintegralCorrection = (LinearintegralConstant * LinearCumulativeerror);
        LinearSlopeofderivitive = Linearerror - Linearlasterror;
        float Linearderivitivecorrection = (LinearSlopeofderivitive * LinearderivitiveConstant);


        float LinearCorrection = LinearproportionalCorrection + LinearintegralCorrection + Linearderivitivecorrection;

        if (LinearCorrection > LinearMaxCorrection) {
            LinearCorrection = LinearMaxCorrection;
        } else if (LinearCorrection < LinearMinCorrection) {
            LinearCorrection = LinearMinCorrection;
        } else LinearCorrection = LinearCorrection;
        return LinearCorrection;

    }

    public void AutonomousHardwareMap() {
        FrontLeft = hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight = hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft = hardwareMap.dcMotor.get(backLeftMotor);
        BackRight = hardwareMap.dcMotor.get(backRightMotor);

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Catapult = hardwareMap.dcMotor.get(catapult);
        BallCollection = hardwareMap.dcMotor.get(ballCollection);
        CapBallLiftLeft = hardwareMap.dcMotor.get(capBallLiftLeft);
        ButtonPusherActuator = hardwareMap.dcMotor.get(buttonPusherActuator);
        ButtonPusherActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        CatapultStop = hardwareMap.touchSensor.get(catapultStop);

        BallControl = hardwareMap.servo.get(ballControll);
        ButtonPusherArm = hardwareMap.servo.get(buttonPusherArm);
        ButtonPusher = hardwareMap.servo.get(buttonPusher);
        CapBallFork = hardwareMap.servo.get(capBallFork);

        CapBallarm1 = hardwareMap.crservo.get(capBallarm1);
        CapBallarm2 = hardwareMap.crservo.get(capBallarm2);

        SideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, sideRangeSensor);
        FrontRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, frontRangeSensor);
        BackRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, backRangeSensor);

        Gyro = hardwareMap.gyroSensor.get(gyroSensor);

        BeaconColorSensor = hardwareMap.colorSensor.get(beaconColorSensor);
        LeftWhiteLineFinder = hardwareMap.opticalDistanceSensor.get(LeftwhiteLineFinder);
        RightWhiteLineFinder=hardwareMap.opticalDistanceSensor.get(RightwhiteLineFinder);
        BeaconColorSensor.enableLed(false);

    }

    public void InitializeServoPositions() {
        if (BallControl != null) {
            BallControl.setPosition(ballControlStartPosition);
        }
        if (ButtonPusher != null) {
            ButtonPusher.setPosition(buttonPusherLeft);
        }

    }

    public void setPower(double FL, double FR, double BL, double BR, double Constant) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR*Constant);
        BackLeft.setPower(BL);
        BackRight.setPower(BR*Constant);
    }

    public void setPower(double FL, double FR, double BL, double BR) {
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR);
        BackLeft.setPower(BL);
        BackRight.setPower(BR);
    }

    public void SetMode(DcMotor.RunMode mode) {
        FrontLeft.setMode(mode);
        FrontRight.setMode(mode);
        BackLeft.setMode(mode);
        BackRight.setMode(mode);
    }

    public void setMaxSpeed(int TicksPerSecond) {
        FrontLeft.setMaxSpeed(TicksPerSecond);
        FrontRight.setMaxSpeed(TicksPerSecond);
        BackLeft.setMaxSpeed(TicksPerSecond);
        BackRight.setMaxSpeed(TicksPerSecond);
    }

    public void setMaxSpeed(int TicksPerSecond, double LeftConstant, double RightConstant) {
        FrontLeft.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*LeftConstant)));
        FrontRight.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*RightConstant)));
        BackLeft.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*LeftConstant)));
        BackRight.setMaxSpeed((int) Math.abs(TicksPerSecond+(TicksPerSecond*RightConstant)));
    }

    public void DriveWithPID(double minPower,double Distance, int TargetAngle, int MaxSpeed){
        double Tolerance = 0;
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        LinearproportionalConstant = (float) 0;
        LinearintegralConstant= (float) 0;
        LinearderivitiveConstant = (float) 0;
        double EncoderTicks=CountsPerInch*Distance;

        setPower(minPower,minPower,minPower,minPower);

        setMaxSpeed(MaxSpeed);

        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getCurrentPosition() + EncoderTicks));
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition() + EncoderTicks));
        BackRight.setTargetPosition((int) (BackRight.getCurrentPosition()+ EncoderTicks));

        setPower(minPower,minPower,minPower,minPower);

        setMaxSpeed(MaxSpeed);

        while(FrontLeft.isBusy()&&FrontRight.isBusy()&&BackLeft.isBusy()&&BackRight.isBusy()&&opModeIsActive()) {

            if (TargetAngle - getIntegratedZValue() > Tolerance) {//Angled to the left of where it should be, Left Power must be increased

                setPower(minPower+(FrontLeft.getPower()+PidPowerAdjustment(TargetAngle)),minPower-(FrontRight.getPower()*PidPowerAdjustment(TargetAngle)),minPower+(BackLeft.getPower()*PidPowerAdjustment(TargetAngle)),minPower-(BackRight.getPower()*PidPowerAdjustment(TargetAngle)));
                setMaxSpeed(MaxSpeed,PidPowerAdjustment(TargetAngle),-PidPowerAdjustment(TargetAngle));

            } else if (TargetAngle - getIntegratedZValue() < Tolerance) {//Angled to the right of where it should be,Right Power must be increased

                setPower(minPower-(FrontLeft.getPower()*PidPowerAdjustment(TargetAngle)),minPower+(FrontRight.getPower()*PidPowerAdjustment(TargetAngle)),minPower-(BackLeft.getPower()*PidPowerAdjustment(TargetAngle)),minPower+(BackRight.getPower()*PidPowerAdjustment(TargetAngle)));

                setMaxSpeed(MaxSpeed,-PidPowerAdjustment(TargetAngle),+PidPowerAdjustment(TargetAngle));

            } else {//Robot is at the angle it should be
            }
            telemetry.addData("Current Time", Totalruntime.seconds());
            telemetry.addData("Current Heading", getIntegratedZValue());
            telemetry.addData("Current Adjustment", PidPowerAdjustment(TargetAngle));
            telemetry.addData("Current Proportional Constant", LinearproportionalConstant);
            telemetry.addData("Current Integral Constant", LinearintegralConstant);
            telemetry.addData("Current Derivitive Constant", LinearderivitiveConstant);
            telemetry.addData("Front Left Power",FrontLeft.getPower());
            telemetry.addData("Front Right Power",FrontRight.getPower());
            telemetry.addData("Back Left Power",BackLeft.getPower());
            telemetry.addData("Back Right Power",BackRight.getPower());
            RobotLog.i("Current Time", Totalruntime.seconds());
            RobotLog.i("Current Heading", getIntegratedZValue());
            RobotLog.i("Current Adjustment", PidPowerAdjustment(TargetAngle));
            RobotLog.i("Current Proportional Constant", LinearproportionalConstant);
            RobotLog.i("Current Integral Constant", LinearintegralConstant);
            RobotLog.i("Current Derivitive Constant", LinearderivitiveConstant);
            RobotLog.i("Front Left Power",FrontLeft.getPower());
            RobotLog.i("Front Right Power",FrontRight.getPower());
            RobotLog.i("Back Left Power",BackLeft.getPower());
            RobotLog.i("Back Right Power",BackRight.getPower());
            telemetry.update();

        }
    }

    public void Drive(double minPower, double Distance, int TargetAngle, double TimeOut, boolean PIDdesired) {
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        int AngleToMaintain = getIntegratedZValue();

        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        double EncoderTicks = CountsPerInch * Distance;
        double difference =TargetAngle- getIntegratedZValue();
        double adjustment = difference*.07;
        FrontLeft.setTargetPosition((int) (FrontLeft.getCurrentPosition() + EncoderTicks));
        FrontRight.setTargetPosition((int) (FrontRight.getTargetPosition() + EncoderTicks));
        BackLeft.setTargetPosition((int) (BackLeft.getCurrentPosition() + EncoderTicks));
        BackRight.setTargetPosition((int) (BackRight.getTargetPosition() + EncoderTicks));
        //SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(minPower,minPower,minPower,minPower,.8);
        setMaxSpeed(2500);

        while ((opModeIsActive()&&!isStopRequested()&&
                FrontLeft.isBusy() &&
                FrontRight.isBusy()) &&
                BackLeft.isBusy() &&
                BackRight.isBusy()
                ) {

            setPower(minPower+adjustment,minPower-adjustment,minPower+adjustment,minPower-adjustment);
            telemetry.addData("FL", FrontLeft.getCurrentPosition());
            telemetry.addData("FR", FrontRight.getCurrentPosition());
            telemetry.addData("BL", BackLeft.getCurrentPosition());
            telemetry.addData("BR", BackRight.getCurrentPosition());
            telemetry.update();
        }
        setPower(0, 0, 0, 0);
        sleep(300);
    }

    public void Turn(double LeftPower,double RightPower, int TargetAngle, boolean Pivot){
        int Start = getIntegratedZValue();
        double leftpower = LeftPower;
        double rightpower = RightPower;
        if (Start<TargetAngle){
            setPower(LeftPower,-RightPower,LeftPower,-RightPower);
        }
        else if (Start>TargetAngle){
            setPower(-LeftPower,RightPower,-LeftPower,RightPower);
        }
        else {setPower(0,0,0,0);}

        while(Math.abs(getIntegratedZValue()-TargetAngle)>3){

            if (getIntegratedZValue()<TargetAngle){
                setPower(leftpower,-rightpower,leftpower,-rightpower);
            }
            else if (getIntegratedZValue()>TargetAngle){
                setPower(-leftpower,rightpower,-leftpower,rightpower);
            }
            else {setPower(0,0,0,0);}

        }setPower(0,0,0,0);
    }

    public void Turn(double power, int TargetAngle, boolean Pivot,String Direction) {
        double FrontLeftTurnPower = 0;
        double FrontRightTurnPower = 0;
        double BackLeftTurnPower = 0;
        double BackRightTurnPower = 0;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
        setMaxSpeed(2000);
        double Adjustment = TurningConstant*(Math.abs(getIntegratedZValue()-TargetAngle));
        double Power = power;
        if (Pivot) {

                if (Direction == "clockwise"){

                    FrontLeftTurnPower = 0;
                    FrontRightTurnPower =-Power ;
                    BackLeftTurnPower =0;
                    BackRightTurnPower =-Power;
                    }

                if (Direction =="counterclockwise"){
                    FrontLeftTurnPower = -Power;
                    FrontRightTurnPower = Power;
                    BackLeftTurnPower = -Power;
                    BackRightTurnPower = Power;}


            }
            else{
            if (Direction == "clockwise"){
            FrontLeftTurnPower = Power;
            FrontRightTurnPower =-Power ;
            BackLeftTurnPower = Power;
            BackRightTurnPower =-Power ;}

            if (Direction =="counterclockwise"){
                FrontLeftTurnPower = -Power;
                FrontRightTurnPower = Power;
                BackLeftTurnPower = -Power;
                BackRightTurnPower = Power;}


        }setPower(FrontLeftTurnPower, FrontRightTurnPower, BackLeftTurnPower,BackRightTurnPower);
            while (opModeIsActive()&&!isStopRequested()&&Math.abs(TargetAngle- getIntegratedZValue() ) > 3);
            {
                setPower(FrontLeftTurnPower, FrontRightTurnPower, BackLeftTurnPower,BackRightTurnPower);idle();
                telemetry.addData(">", "Turning!");
                telemetry.update();

            }
            setPower(0, 0, 0, 0);
    }

    public void DriveToWall(int Distance,double Power) {
        while (FrontRangeSensor.getDistance(DistanceUnit.INCH) > Distance) {
            setPower(Power, Power, Power, Power,.8);
        }
        setPower(0, 0, 0, 0);
    }

    public void FindWhiteLine(OpticalDistanceSensor LeftODS,OpticalDistanceSensor RightODS,double Power,double Reflectance) {
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double LeftPower;
        double RightPower;
        runtime.reset();
        while (opModeIsActive()&&!isStopRequested()&&LeftODS.getLightDetected()<.6&&RightODS.getLightDetected()>.6) {
            if(runtime.seconds()>1.5){
                if (LeftODS.getLightDetected()<Reflectance){LeftPower=Power;}else{LeftPower=0;}
                if (RightODS.getLightDetected()<Reflectance){RightPower=Power;}else{RightPower=0;}
                setPower(-LeftPower, -RightPower, -LeftPower, -RightPower,.8);
            }
            else{
                if (LeftODS.getLightDetected()<Reflectance){LeftPower=Power;}else{LeftPower=0;}
                if (RightODS.getLightDetected()<Reflectance){RightPower=Power;}else{RightPower=0;}
                setPower(LeftPower, RightPower, LeftPower, RightPower,.8);
            }
        }
        Gyro.resetZAxisIntegrator();
        setPower(0,0,0,0);
    }



    public void AutonomousButtonPush(String Color) {
        while(!ButtonPusherMax.isPressed()){ButtonPusherActuator.setPower(.5);
            if (!ButtonPusherMin.isPressed()){
            ButtonPusher.setPosition(buttonPusherCenter);}}
        ButtonPusherActuator.setPower(0);
        sleep(500);
        if( Color.toLowerCase()=="red"){
            if (BeaconColorSensor.red()>BeaconColorSensor.blue()){
            ButtonPusher.setPosition(buttonPusherLeft);}
            else if (BeaconColorSensor.blue()>BeaconColorSensor.red()){
                ButtonPusher.setPosition(buttonPusherRight);}
            else {
                RobotLog.a("Beacon Color Not Found");
            }
        }
        if (Color.toLowerCase()=="blue"){
            if (BeaconColorSensor.blue()>BeaconColorSensor.red()){
            ButtonPusher.setPosition(buttonPusherLeft);}
        else if (BeaconColorSensor.red()>BeaconColorSensor.blue()){
            ButtonPusher.setPosition(buttonPusherRight);}
        else {
            RobotLog.a("Beacon Color Not Found");
        }}
        sleep(300);
        while(!ButtonPusherMin.isPressed()){
            ButtonPusherActuator.setPower(-.5);
        }ButtonPusherActuator.setPower(0);

    }

    public void AutonomousBallShoot(){
        while (!CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
        while (CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
        while (!CatapultStop.isPressed()){Catapult.setPower(1);}Catapult.setPower(0);
    }
    public void AutonomousShoot2Balls(){
        if (opModeIsActive()){
            BallControl.setPosition(ballControlStartPosition);
            AutonomousBallShoot();
            BallCollection.setPower(.5);
            BallControl.setPosition(ballControlEngagedPosition);
            sleep(2000);
            BallControl.setPosition(ballControlStartPosition);
            AutonomousBallShoot();

        }
    }

}


    /*

    public void InitializeVuforia(){
        parameters=new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey=VuforiaLicenseKey;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer= ClassFactory.createVuforiaLocalizer(parameters);
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016_17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        Wheels = visionTargets.get(0);
        Wheels.setName("Wheels Target");
        Wheels.setLocation(createMatrix(144*mmPerInch,60*mmPerInch,6*mmPerInch,0,0,0));

        Gears = visionTargets.get(1);
        Gears.setName("Gears Target");
        Gears.setLocation(createMatrix(60*mmPerInch,144*mmPerInch,6*mmPerInch,0,0,0));

        Tools = visionTargets.get(2);
        Tools.setName("Tools Target");
        Tools.setLocation(createMatrix(60+48*mmPerInch,144*mmPerInch,6*mmPerInch,0,0,0));

        Legos = visionTargets.get(3);
        Legos.setName("Legos Target");
        Legos.setLocation(createMatrix(144*mmPerInch,60+48*mmPerInch,0*mmPerInch,0,0,0));

        phoneLocation = createMatrix(9*mmPerInch,9*mmPerInch,14*mmPerInch,0,0,0);

        Wheelslistener = (VuforiaTrackableDefaultListener) Wheels.getListener();
        Wheelslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        Gearslistener = (VuforiaTrackableDefaultListener) Gears.getListener();
        Gearslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        Toolslistener = (VuforiaTrackableDefaultListener) Tools.getListener();
        Toolslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        Legoslistener = (VuforiaTrackableDefaultListener) Legos.getListener();
        Legoslistener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

        lastKnownLocation = createMatrix(0,0,0,0,0,0);
    }
    OpenGLMatrix LatestLocation =null;
    double RobotX=0;
    double RobotY=0;
    double RobotTheta=0;
    float Output = Float.parseFloat(null);

    public double RobotLocationOnField(String LookingFor, String Tracking){

        switch(Tracking.toLowerCase()){
            case "wheels":{
                LatestLocation=Wheelslistener.getUpdatedRobotLocation();
            }break;
            case "gears":{
                LatestLocation=Gearslistener.getUpdatedRobotLocation();
            }break;
            case "tools":{
                LatestLocation=Toolslistener.getUpdatedRobotLocation();
            }break;
            case "legos":{
                LatestLocation=Legoslistener.getUpdatedRobotLocation();
            }break;
        }
        if (LatestLocation !=null){
            LatestLocation = lastKnownLocation;}
        float[] Coordinates = lastKnownLocation.getTranslation().getData();

        RobotX = Coordinates[0];
        RobotY = Coordinates[1];
        RobotTheta = Orientation.getOrientation(lastKnownLocation,AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;


                switch (LookingFor.toLowerCase()){
                    case"x":{
                        Output= (float) RobotX;
                    }break;
                    case"y":{
                        Output = (float) RobotY;
                    }break;
                    case"theta":{
                        Output = (float) RobotTheta;
                    }break;
                }
        return Output;
        }

    public void VuforiaTelemetry(){
        telemetry.addData("Tracking"+Wheels.getName(),Wheelslistener.isVisible());
        telemetry.addData("Last Known Loacation", formatMatrix(lastKnownLocation));
        telemetry.update();
    }

    public OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w){
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ, AngleUnit.DEGREES,u,v,w));
    }
    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }*/



