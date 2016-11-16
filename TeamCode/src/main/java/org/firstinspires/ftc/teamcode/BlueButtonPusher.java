package com.qualcomm.ftcrobotcontroller.opmodes;

import android.hardware.Sensor;
import android.hardware.SensorEvent;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BLUE_B_AUTONOMOUS extends OpMode {
    UltrasonicSensor Sonic;
    ColorSensor FrontCS;
    private enum State {
        STATE_INITIAL,
        STATE_DRIVE_TO_BEACON,
        STATE_FOLLOW_LINE,
        STATE_SQUARE_TO_WALL,
        STATE_DEPLOY_CLIMBERS,
        STATE_DRIVE_TO_MOUNTAIN,
        STATE_CLIMB_MOUNTAIN,
        stop,
    }

    double A = .8;

    private final PathSeg[] mBeaconPath = {
            new PathSeg(.5, 0, 24, DriveStyle.Linear),
            new PathSeg(.25, 0, 12, DriveStyle.Linear),
            new PathSeg(.125, 0, 6, DriveStyle.Linear),
            new PathSeg(.2, -45, 0, DriveStyle.CounterClockwise),
            new PathSeg(.5, 0, 16, DriveStyle.Linear),
            new PathSeg(.25, 0, 8, DriveStyle.Linear),
            new PathSeg(.125, 0, 4, DriveStyle.Linear),/*
            new PathSeg(.2, 0, 6, DriveStyle.Linear),
            new PathSeg(.2, -90, 0, DriveStyle.Clockwise),
            new PathSeg(.2, 0, 6, DriveStyle.Linear),
            new PathSeg(.2, 0, 0, DriveStyle.CounterClockwise),
            new PathSeg(.2, 0, -6, DriveStyle.Linear),*/

    };


    private GyroSensor Gyro;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private int FrontLeftEncoderTarget;
    private int BackLeftEncoderTarget;
    private int FrontRightEncoderTarget;
    private int BackRightEncoderTarget;
    private int Heading;
    private final static int Encoder_CPR = 1440;
    private final static int WheelDiameter = 4;
    private final static double Circumference = Math.PI * WheelDiameter;
    private final static double Rotations = 1 / Circumference;
    private final static double CountsPerInch = Encoder_CPR * Rotations;
    private final ElapsedTime mRuntime = new ElapsedTime();
    private final ElapsedTime mStateTime = new ElapsedTime();
    private State mCurrentState;
    private PathSeg[] mCurrentPath;
    private int mCurrentSeg;
    public DriveStyle mdirection;
    public double UltrasonicTarget;
    enum DriveStyle {Linear, Clockwise, CounterClockwise, UltraSonic}

    public enum Color {Red,Blue,Green}
    public Color color;
    //enum DriveStyle {Mechanum, TankDrive, OneJoystickDrive}

    private DriveStyle mDirection;
    public BLUE_B_AUTONOMOUS() {
    }

    @Override
    public void init() {
        // Initialize class members.
        Gyro = hardwareMap.gyroSensor.get("Gyro");
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Sonic  = hardwareMap.ultrasonicSensor.get("Sonic");
        FrontCS = hardwareMap.colorSensor.get("FCS");
        setDrivePower(0, 0, 0, 0);
        calibrate();

        FrontCS.enableLed(true);
        resetDriveEncoders();
    }

    public void init_loop() {
        initialize();
    }

    @Override
    public void start() {
        setDriveSpeed(0, 0, 0, 0);
        runToPosition();
        mRuntime.reset();
        newState(State.STATE_INITIAL);
        initialize();


    }

    @Override
    public void loop() {
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());
        telemetry.addData("Integrated Z", getIntegratedZValue());
        telemetry.addData("Targetheading(?)", Heading);
        telemetry.addData("1", String.format("FL %5d - FR %5d - BL %5d - BR %5d", getFrontLeftPosition(),
                getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition()));
        telemetry.addData("turnComplete", turnComplete());
        telemetry.addData("MoveComplete", MoveComplete());
        telemetry.addData("Direction", mDirection);
        telemetry.addData("US", Sonic.getUltrasonicLevel());

        switch (mCurrentState) {

            case STATE_INITIAL:
                if (encodersAtZero() && !Gyro.isCalibrating() && FrontRed()) {
                    startPath(mBeaconPath);
                    newState(State.STATE_DRIVE_TO_BEACON);
                    //
                    // Climber.setPosition(.8);
                }
                break;
            case STATE_DRIVE_TO_BEACON:
                if (pathComplete()) {
                    newState(State.STATE_DRIVE_TO_MOUNTAIN);
                }
                break;
            case STATE_DRIVE_TO_MOUNTAIN:

                newState(State.stop);
                break;
            case stop: {
                useConstantPower();
                setDrivePower(0, 0, 0, 0);
            }


        }
    }


    @Override
    public void stop() {
        useConstantPower();
        setDrivePower(0, 0, 0, 0);
    }

    private void newState(State newState) {
        mStateTime.reset();
        mCurrentState = newState;
    }
    public boolean FrontRed(){
        return ((FrontCS.red()>FrontCS.blue()) && (FrontCS.red()>FrontCS.green()));
    }
    public boolean FrontBlue(){
        return ((FrontCS.blue()>FrontCS.red()) && (FrontCS.blue()>FrontCS.green()));
    }
    public boolean FrontGreen(){
        return ((FrontCS.green()>FrontCS.blue()) && (FrontCS.green()>FrontCS.red()));
    }
    public void DriveUntil(Color mcolor, double Power){
        switch (mcolor){
            case Red: {
                setDrivePower(Power,Power,Power,Power);
                if (FrontRed()){setDrivePower(0,0,0,0);}
            }
            case Blue: {
                setDrivePower(Power,Power,Power,Power);
                if (FrontBlue()){setDrivePower(0,0,0,0);}
            }
            case Green: {
                setDrivePower(Power,Power,Power,Power);
                if (FrontGreen()){setDrivePower(0,0,0,0);}
            }


        }

    }
    public void DriveUntilUltraSonicTargetHit(double Power, int Target){
     SetUltrasonicTarget(Target);
     setDrivePower(Power,Power,Power,Power);
     if (UltrasonicTargetHit()){setDrivePower(0,0,0,0);}
    }

    private int getFrontLeftPosition() {
        return FrontLeft.getCurrentPosition();
    }

    private int getFrontRightPosition() {
        return FrontRight.getCurrentPosition();
    }

    private int getBackLeftPosition() {
        return BackLeft.getCurrentPosition();
    }

    private int getBackRightPosition() {
        return BackRight.getCurrentPosition();
    }

    private int getIntegratedZValue() {// Fixes the problematic wrap around from 0 to 359.
        int heading = Gyro.getHeading();
        if (heading > 180) {
            heading -= 360;
        }
        return heading;
    }

    private void syncEncoders() {// initialize the publically accessable value "MotorEncoderTarget" for access through out the class.
        FrontLeftEncoderTarget = FrontLeft.getCurrentPosition();
        FrontRightEncoderTarget = FrontRight.getCurrentPosition();
        BackLeftEncoderTarget = BackLeft.getCurrentPosition();
        BackRightEncoderTarget = BackRight.getCurrentPosition();
    }

    private void setEncoderTarget(int FrontleftEncoder, int FrontrightEncoder, int BackleftEncoder, int BackrightEncoder) { // Changes the publically accessable "MoterEncoderTarget" to the target for access through out the class.
        FrontLeft.setTargetPosition(FrontLeftEncoderTarget = FrontleftEncoder);
        FrontRight.setTargetPosition(FrontRightEncoderTarget = FrontrightEncoder);
        BackLeft.setTargetPosition(BackLeftEncoderTarget = BackleftEncoder);
        BackRight.setTargetPosition(BackRightEncoderTarget = BackrightEncoder);
    }

    private void addEncoderTarget(int FrontleftEncoder, int FrontrightEncoder, int BackleftEncoder, int BackrightEncoder) { // Adds the new target to the old target for access through out the class./
        FrontLeft.setTargetPosition((FrontLeftEncoderTarget += FrontleftEncoder));
        FrontRight.setTargetPosition(FrontRightEncoderTarget += FrontrightEncoder);
        BackLeft.setTargetPosition(BackLeftEncoderTarget += BackleftEncoder);
        BackRight.setTargetPosition(BackRightEncoderTarget += BackrightEncoder);

    }

    public void SetUltrasonicTarget(double Distance){
        UltrasonicTarget = Distance;
    }

    public void InitializeUltrasonicLevel(){
        UltrasonicTarget = Sonic.getUltrasonicLevel();
    }

    public boolean UltraSonicTargetHit( int Target){
        return (UltrasonicTarget<Target);
    }

    private void syncHeading() {// initalize heading for access through out the class
        Heading = getIntegratedZValue();
    }

    private void setTargetHeading(int TargetHeading) { // change heading to target heading for access through out the class
        Heading = TargetHeading;
    }

    private void setDirection(DriveStyle direction) {// sets a new direction and makes said direction publically accessable. This is used for the TurnComplete and MoveComplete methods.
        mDirection = direction;
    }

    private void initializeDirection() {// Initializes the direction as linear to avoid any Null Pointer exceptions or out of bound array issues.
        mDirection = DriveStyle.Linear;
    }

    public void initialize(){
        initializeDirection();
        InitializeUltrasonicLevel();
        syncEncoders();
        syncHeading();
        runToPosition();
    }

    public void calibrate(){
        resetDriveEncoders();
        Gyro.calibrate();

    }

    private void setDrivePower(double Frontleftpower, double Frontrightpower, double Backleftpower, double Backrightpower) { // Set power with clipped range to avoid any overloading.
        FrontLeft.setPower(Range.clip(Frontleftpower, -1, 1));
        FrontRight.setPower(Range.clip(Frontrightpower, -1, 1));
        BackLeft.setPower(Range.clip(Backleftpower, -1, 1));
        BackRight.setPower(Range.clip(Backrightpower, -1, 1));
    }

    private void setDriveSpeed(double Frontleft, double Frontright, double Backleft, double Backright) { 
        setDrivePower(Frontleft, Frontright, Backleft, Backright);
    }

    private void PowerWithAngularAdjustment(double Power, double TargetAngle) {// Uses the gyro to drive straight
        double difference = (Math.abs(TargetAngle- Gyro.getHeading()));
        double Adjustment = 1.02 * difference;
        if (getIntegratedZValue() - TargetAngle < -1) {
            setDrivePower(Power / Adjustment, Power * Adjustment, Power / Adjustment, Power * Adjustment);
        } else if (getIntegratedZValue() - TargetAngle > 1) {
            setDrivePower(Power * Adjustment, Power / Adjustment, Power * Adjustment, Power / Adjustment);
        } else {
            setDrivePower(Power, Power, Power, Power);
        }
    }

    private void runToPosition() {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    private void useConstantSpeed() {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    private void useConstantPower() {setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);}

    private void resetDriveEncoders() {
        setEncoderTarget(0, 0, 0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    private void setDriveMode(DcMotorController.RunMode mode) {// Ensure that motor drive mode is the correct moce and if not set the drive mode to the correct mode.
        if (FrontLeft.getMode() != mode)
            FrontLeft.setMode(mode);

        if (FrontRight.getMode() != mode)
            FrontRight.setMode(mode);
        if (BackLeft.getMode() != mode)
            BackLeft.setMode(mode);

        if (BackRight.getMode() != mode)
            BackRight.setMode(mode);
    }

    private boolean encodersAtZero() {// ensure that encoders have reset
        return ((Math.abs(getFrontLeftPosition()) < 5) && (Math.abs(getFrontRightPosition()) < 5) && (Math.abs(getBackLeftPosition()) < 5) && (Math.abs(getBackRightPosition()) < 5));
    }

    private void EncoderStop() {// Stop encoders wehn sensor has been activated and target position has been set to an unreachable number.
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition());
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition());
        BackLeft.setTargetPosition(BackLeft.getCurrentPosition());
        BackRight.setTargetPosition(BackRight.getCurrentPosition());
    }

    private void DriveWithEncoder(double Power, int TargetAngle, double Distance, DriveStyle Direction) {// use encoders and gyro drive and turn.
        double power = Range.clip(Power, 0, .2);
        setDirection(Direction);
        switch (mDirection) {
            case Linear: {
                int counts = (int) (Distance * CountsPerInch);
                setDrivePower(Power,Power,Power,Power);
                addEncoderTarget(counts, counts, counts, counts);
                setTargetHeading(TargetAngle);

            }
            break;
            case Clockwise: {
                setDrivePower(power, power, power, power);
                setTargetHeading(TargetAngle);

                if (Math.abs(getIntegratedZValue() - Heading) < 1.25) {
                    setEncoderTarget(getFrontLeftPosition(), getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition());
                } else
                    addEncoderTarget(1000000, -1000000, 100000, -100000);// target position set to unreachable number to keep the drive mode as run to position constantly. once the target angle has been reached the encoder target will be set to the current position

            }

            break;
            case CounterClockwise: {
                setTargetHeading(TargetAngle);
                setDrivePower(power, power, power, power);
                if (Math.abs(getIntegratedZValue() - Heading) < 1.25) {
                    setEncoderTarget(getFrontLeftPosition(), getFrontRightPosition(), getBackLeftPosition(), getBackRightPosition());
                } else
                    addEncoderTarget(-1000000, 1000000, -100000, 100000);

            }
            break;
           /* case UltraSonic:{
                setTargetHeading(TargetAngle);
                setDrivePower(power,power,power,power);
                SetUltrasonicTarget(Distance);
                if (Sonic.getUltrasonicLevel()<UltrasonicTarget){EncoderStop(); syncEncoders();}
                else {addEncoderTarget(100000,100000,100000,100000);}
            }*/
        }
    }

    private boolean turnComplete() {
        if (((Math.abs(getIntegratedZValue() - Heading) < 1.25) && mDirection != DriveStyle.Linear) && mDirection != DriveStyle.UltraSonic) {
            EncoderStop();
            syncEncoders();
        }
        return ((Math.abs(getIntegratedZValue() - Heading) < 1.25) && mDirection != DriveStyle.Linear && mDirection != DriveStyle.UltraSonic);
    }

    private boolean MoveComplete() {
        return ((!FrontLeft.isBusy() && !FrontRight.isBusy() && !BackLeft.isBusy() && !BackRight.isBusy()) && (mDirection == DriveStyle.Linear));
    }

    private boolean UltrasonicTargetHit(){
        if ((Sonic.getUltrasonicLevel()<UltrasonicTarget)&&mDirection == DriveStyle.UltraSonic){
            EncoderStop();
            syncEncoders();}
        return ((Sonic.getUltrasonicLevel()<UltrasonicTarget)&&mDirection == DriveStyle.UltraSonic);
        }

    private void startPath(PathSeg[] path) {
        mCurrentPath = path;
        mCurrentSeg = 0;
        initialize();
        startSeg();
    }

    private void startSeg() {

        if (mCurrentPath != null) {
            DriveWithEncoder(mCurrentPath[mCurrentSeg].mpower, mCurrentPath[mCurrentSeg].mtargetangle, mCurrentPath[mCurrentSeg].mdistance, mCurrentPath[mCurrentSeg].mdirection);

            mCurrentSeg++;
        }
    }

    private boolean pathComplete() {

        if (MoveComplete() || turnComplete() || UltrasonicTargetHit()) {


            if (mCurrentSeg < mCurrentPath.length) {
                startSeg();
            } else {
                mCurrentPath = null;
                mCurrentSeg = 0;
                initialize();
                setDriveSpeed(0, 0, 0, 0);
                useConstantSpeed();
                return true;
            }


        }
        return false;
    }
}

class PathSeg {
    double mpower;
    int mtargetangle;
    double mdistance;
    BLUE_B_AUTONOMOUS.DriveStyle mdirection;

    public PathSeg(double Power, int TargetAngle, double Distance, BLUE_B_AUTONOMOUS.DriveStyle direction) {
        mpower = Power;
        mtargetangle = TargetAngle;
        mdistance = Distance;
        mdirection = direction;
    }
}
