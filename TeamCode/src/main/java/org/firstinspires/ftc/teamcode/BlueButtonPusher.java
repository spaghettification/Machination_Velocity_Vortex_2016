package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by Trevor on 10/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueButtonPusher", group = "6994 Bot")

public class BlueButtonPusher extends LinearHardwareMap {
    private ElapsedTime runtime=new ElapsedTime();
    public double Distance_From_Wall=47;
    public double DistanceFromWallToCenterOfRobot=Distance_From_Wall+8;
    public double FirstPushOff=8;
    public int Distancefromwalltowhitetape=36;
    public int xbuffer=12;
    public int earlyStop=6;
    public double totalXTravel=DistanceFromWallToCenterOfRobot-xbuffer;
    public double totalYTravel = Distancefromwalltowhitetape-earlyStop-FirstPushOff;
    public double Theta=Math.atan((totalYTravel)/(totalXTravel));
    public double Hypotenuse=Math.sqrt(totalXTravel*totalXTravel+totalYTravel*totalYTravel);
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        sleep(2000);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.125,0,3000,FirstPushOff,4);
        DriveWithEncoders(DriveSyle.ClockwiseWithGyroScope,.125, (int) Theta,0,0,2);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.125, (int) Theta,2000,Hypotenuse,4);
        DriveWithEncoders(DriveSyle.CounterClockwiseWithGyroScope,.125,0,2000,0,2);


    }
    public boolean WhiteLineFound() {
        return false;
    }
    public void initialize(){
        FrontLeft=hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight=hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft=hardwareMap.dcMotor.get(backLeftMotor);
        BackRight=hardwareMap.dcMotor.get(backRightMotor);
        Gyro=hardwareMap.gyroSensor.get(gyroSensor);


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(2000);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDriveMotorMode(DcMotor.RunMode Mode){
        FrontLeft.setMode(Mode);
        FrontRight.setMode(Mode);
        BackLeft.setMode(Mode);
        BackRight.setMode(Mode);
    }
    public void SetDrivePower(double FrontLeftPower,double FrontRightPower, double BackLeftPower,double BackRightPower){
        FrontLeft.setPower(FrontLeftPower);
        FrontRight.setPower(FrontRightPower);
        BackLeft.setPower(BackLeftPower);
        BackRight.setPower(BackRightPower);
    }
    public void setTargetPosition(int FrontLeftTarget, int FrontRightTarget, int BackLeftTarget, int BackRightTarget){
        FrontLeft.setTargetPosition(FrontLeftTarget);
        FrontRight.setTargetPosition(FrontRightTarget);
        BackLeft.setTargetPosition(BackLeftTarget);
        BackRight.setTargetPosition(BackRightTarget);
    }

    enum DriveSyle{LinearWithEnocders, ClockwiseWithGyroScope,CounterClockwiseWithGyroScope}


    float Linearlasterror=0;
    float LinearCumulativeerror=0;
    float Angularlasterror=0;
    float AngularCumulativeerror=0;
    public void DriveWithEncoders(DriveSyle driveSyle, double Power,int TargetAngle, int MaxSpeed, double Distance, double Pause){
        int FrontLeftCurrent = FrontLeft.getCurrentPosition();
        int FrontRightCurrent = FrontRight.getCurrentPosition();
        int BackLeftCurrent = BackLeft.getCurrentPosition();
        int BackRightCurrent = BackRight.getCurrentPosition();
        float LinearproportionalCorrection;
        float LinearintegralCorrection;
        float LinearSlopeofderivitive;
        float LinearMaxCorrection=100;
        float LinearMinCorrection=15;
        float AngularproportionalCorrection;
        float AngularintegralCorrection;
        float AngularSlopeofderivitive;
        float AngularMaxCorrection=100;
        float AngularMinCorrection=15;
        String Report;
        switch (driveSyle){
            case LinearWithEnocders:{

                if (opModeIsActive()){
                    int FrontLeftTarget= (FrontLeftCurrent+(int)(Distance*CountsPerInch));
                    int FrontRightTarget= (FrontRightCurrent+(int)(Distance*CountsPerInch));
                    int BackLeftTarget= (BackLeftCurrent+(int)(Distance*CountsPerInch));
                    int BackRightTarget= (BackRightCurrent+(int)(Distance*CountsPerInch));

                    FrontLeft.setTargetPosition(FrontLeftTarget);
                    FrontRight.setTargetPosition(FrontRightTarget);
                    BackLeft.setTargetPosition(BackLeftTarget);
                    BackRight.setTargetPosition(BackRightTarget);
                    setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

                    runtime.reset();

                    FrontLeft.setPower(Power);
                    FrontRight.setPower(Power);
                    BackLeft.setPower(Power);
                    BackRight.setPower(Power);

                    while (  opModeIsActive()&& FrontLeft.isBusy()
                            &&FrontRight.isBusy()
                            &&BackLeft.isBusy()
                            &&BackRight.isBusy()
                            &&runtime.seconds()<Pause)
                    {
                        float Linearerror=Math.abs(TargetAngle-getIntegratedZValue());
                        LinearproportionalCorrection= (float) (LinearproportionalConstant*Linearerror);
                        LinearCumulativeerror+=Linearerror;
                        LinearintegralCorrection= (float) (LinearintegralConstant*LinearCumulativeerror);
                        LinearSlopeofderivitive=Linearerror-Linearlasterror;
                        float Linearderivitivecorrection= (float) (LinearSlopeofderivitive*LinearderivitiveConstant);


                        float LinearCorrection=LinearproportionalCorrection+LinearintegralCorrection+Linearderivitivecorrection;

                        if (LinearCorrection>LinearMaxCorrection){LinearCorrection=LinearMaxCorrection;}

                        else if (LinearCorrection<LinearMinCorrection){LinearCorrection=LinearMinCorrection;}

                        else LinearCorrection=LinearCorrection;
                        if (getIntegratedZValue()>TargetAngle){
                            FrontLeft.setPower(Power-LinearCorrection);
                            FrontRight.setPower(Power+LinearCorrection);
                            BackLeft.setPower(Power-LinearCorrection);
                            BackRight.setPower(Power+LinearCorrection);
                        }
                        else if (TargetAngle<getIntegratedZValue()){
                            FrontLeft.setPower(Power+LinearCorrection);
                            FrontRight.setPower(Power-LinearCorrection);
                            BackLeft.setPower(Power+LinearCorrection);
                            BackRight.setPower(Power-LinearCorrection);}
                        else {
                            FrontLeft.setPower(Power);
                            FrontRight.setPower(Power);
                            BackLeft.setPower(Power);
                            BackRight.setPower(Power);
                        }float Linearlasterror=Linearerror;

                        telemetry.addData("FrontLeft",FrontLeft.getCurrentPosition());
                        telemetry.addData("FrontRight",FrontRight.getCurrentPosition());
                        telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
                        telemetry.addData("BackRight",BackRight.getCurrentPosition());

                        telemetry.update();
                        //Add Data to the text document already formatted and ready to be converted to CSV then imported to excel.*

                        /*GyroTracker.print(getIntegratedZValue()+",");
                        GyroTracker.print(LinearproportionalCorrection+",");
                        GyroTracker.print(LinearintegralCorrection+",");
                        GyroTracker.print(Linearderivitivecorrection+",");
                        GyroTracker.print(runtime.seconds()+",");
                        GyroTracker. println();*/
                    }
                    FrontLeft.setPower(0);
                    FrontRight.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                    setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER );

                }

            }break;

            case ClockwiseWithGyroScope:{
                if (opModeIsActive()){

                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    runtime.reset();
                    while (Math.abs(getIntegratedZValue()-TargetAngle)<1.2&&runtime.seconds()>Pause){
                        float Angularerror=Math.abs(TargetAngle-getIntegratedZValue());
                        AngularproportionalCorrection=AngularproportionalConstant*Angularerror;
                        AngularCumulativeerror+=Angularerror;
                        AngularintegralCorrection=AngularintegralConstant*AngularCumulativeerror;
                        AngularSlopeofderivitive=Angularerror-Angularlasterror;
                        float Angularderivitivecorrection=AngularSlopeofderivitive*AngularderivitiveConstant;


                        float AngularCorrection=AngularproportionalCorrection+AngularintegralCorrection+Angularderivitivecorrection;

                        if (AngularCorrection>AngularMaxCorrection){AngularCorrection=AngularMaxCorrection;}

                        else if (AngularCorrection<AngularMinCorrection){AngularCorrection=AngularMinCorrection;}

                        else {AngularCorrection=AngularCorrection;}
                        FrontLeft.setPower(Power-AngularCorrection);
                        FrontRight.setPower(-Power+AngularCorrection);
                        BackLeft.setPower(Power-AngularCorrection);
                        BackRight.setPower(-Power+AngularCorrection);
                        float Angularlasterror=Angularerror;

                    }

                }
            }break;

            case CounterClockwiseWithGyroScope:{
                if (opModeIsActive()){

                    setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    runtime.reset();
                    while (Math.abs(getIntegratedZValue()-TargetAngle)<1.2&&runtime.seconds()>Pause){
                        float Angularerror=Math.abs(TargetAngle-getIntegratedZValue());
                        AngularproportionalCorrection=AngularproportionalConstant*Angularerror;
                        AngularCumulativeerror+=Angularerror;
                        AngularintegralCorrection=AngularintegralConstant*AngularCumulativeerror;
                        AngularSlopeofderivitive=Angularerror-Angularlasterror;
                        float Angularderivitivecorrection=AngularSlopeofderivitive*AngularderivitiveConstant;
                        float AngularCorrection=AngularproportionalCorrection+AngularintegralCorrection+Angularderivitivecorrection;
                        if (AngularCorrection>AngularMaxCorrection){AngularCorrection=AngularMaxCorrection;}
                        else if (AngularCorrection<AngularMinCorrection){AngularCorrection=AngularMinCorrection;}
                        else {AngularCorrection=AngularCorrection;}
                        FrontLeft.setPower(-Power+AngularCorrection);
                        FrontRight.setPower(Power-AngularCorrection);
                        BackLeft.setPower(-Power+AngularCorrection);
                        BackRight.setPower(Power-AngularCorrection);
                        float Angularlasterror=Angularerror;
                    }


                }
            }break;
        }
    }
}
