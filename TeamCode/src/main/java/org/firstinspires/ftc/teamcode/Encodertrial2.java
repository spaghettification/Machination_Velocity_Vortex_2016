package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

/**
 * Created by Trevor on 11/6/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "2", group = "6994 Bot")
public class Encodertrial2 extends LinearHardwareMap {
    ElapsedTime runtime= new ElapsedTime();
    public float Linearlasterror;
    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft           =hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight          =hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft            =hardwareMap.dcMotor.get(backLeftMotor);
        BackRight           =hardwareMap.dcMotor.get(backRightMotor);
        Gyro                =hardwareMap.gyroSensor.get(gyroSensor);
        BeaconColorSensor   =hardwareMap.ColorSensor.get(beaconColorSensor);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Gyro.calibrate();

        /*while(!Gyro.isCalibrating() && opModeIsActive()){
            SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData(">","Calibrating Gyro");
            telemetry.update();
            idle();
            sleep(50);
            telemetry.addData(">", "Ready!");
            telemetry.addData(">", "Hey Jason, Try not to Fuck up");
            telemetry.update();*/
        }
        waitForStart();
        while(!Gyro.isCalibrating() && opModeIsActive()){
            SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData(">","Calibrating Gyro");
            telemetry.update();
            idle();
            sleep(50);
            telemetry.addData(">", "Ready!");
            telemetry.addData(">", "Hey Jason, Try not to Fuck up");
            telemetry.update();
        }
        Drive(.125,12,0,5,false);
        Drive(.125,24,0,3,false);
        Drive(-.125,-12,0,3,false);
    }
    
    public void Drive(double minPower, int Distance,int TargetAngle, double TimeOut,boolean PIDdesired){
        double FrontLeftDynamicPower;
        double FrontRightDynamicPower;
        double BackLeftDynamicPower;
        double BackRightDynamicPower;
        int AngleToMaintain=getIntegratedZValue();

        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep (200);
        //SetMode(DcMotor.RunMode.RUN_TO_POSITION);
        double EncoderTicks=Distance*4*Math.PI/1120;/*
        while ((opModeIsActive()&&
                FrontLeft.isBusy()&&
                FrontRight.isBusy()&&
                BackLeft.isBusy()&&
                BackRight.isBusy())
                ||(runtime.seconds()<TimeOut&&
                opModeIsActive())){
            */

        while (((Math.abs(FrontLeft.getCurrentPosition())<EncoderTicks&&/*
                Math.abs(FrontRight.getCurrentPosition())<EncoderTicks&&
                Math.abs(BackLeft.getCurrentPosition())<EncoderTicks&&
                Math.abs(BackRight.getCurrentPosition())<EncoderTicks)&&*/
                opModeIsActive()))||(runtime.seconds()<TimeOut&&opModeIsActive())){

            if (getIntegratedZValue()>TargetAngle&&PIDdesired) {//VeeringRight

                FrontLeftDynamicPower    = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                FrontRightDynamicPower   = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
                BackLeftDynamicPower     = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                BackRightDynamicPower    = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);

            } else if(getIntegratedZValue()<TargetAngle&&PIDdesired){//VeeringLeft

                FrontLeftDynamicPower    = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
                FrontRightDynamicPower   = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
                BackLeftDynamicPower     = Range.clip(minPower + PidPowerAdjustment(AngleToMaintain), 0, 1);
                BackRightDynamicPower    = Range.clip(minPower - PidPowerAdjustment(AngleToMaintain), 0, 1);
            } else {

                FrontLeftDynamicPower    = Range.clip(minPower, 0, 1);
                FrontRightDynamicPower   = Range.clip(minPower, 0, 1);
                BackLeftDynamicPower     = Range.clip(minPower, 0, 1);
                BackRightDynamicPower    = Range.clip(minPower, 0, 1);}

            setPower(FrontLeftDynamicPower,FrontRightDynamicPower,BackLeftDynamicPower,BackRightDynamicPower);
            idle();
            sleep(50);
        }
        setPower(0,0,0,0);
        sleep(300);
        SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Turn(double Power, int TargetAngle){
        double FrontLeftTurnPower   = 0;
        double FrontRightTurnPower  = 0;
        double BackLeftTurnPower    = 0;
        double BackRightTurnPower   = 0;
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(300);
        if (TargetAngle>getIntegratedZValue()){
            FrontLeftTurnPower      =  Power;
            FrontRightTurnPower     =- Power;
            BackLeftTurnPower       =  Power;
            BackRightTurnPower      =- Power;

        }else if (TargetAngle<getIntegratedZValue()){
            FrontLeftTurnPower  =- Power;
            FrontRightTurnPower =  Power;
            BackLeftTurnPower   =- Power;
            BackRightTurnPower  =  Power;
        }
        else{setPower(0,0,0,0);}
        do {

        }
        while (Math.abs(getIntegratedZValue()-TargetAngle)>1.25);{
            setPower(FrontLeftTurnPower,FrontRightTurnPower,BackLeftTurnPower,BackRightTurnPower);
            idle();
            telemetry.addData(">", "Turning!");
            sleep(50);

        }
        setPower(0,0,0,0);
    }

    public void pressButton(string TeamColor, double LeftButtonPusherEngagedPosition,double RightButtonPusherEngagedPosition,double LeftButtonPusherStartPosition,double RightButtonPusherStartPosition,){
    if (BeaconColorSensor.Blue();>BeaconColorSensor.Red()&&BeaconColorSensor.Blue()>BeaconColorSensor.Red()){
        switch(TeamColor.toLowerCase){
             case "blue":{
                 if (BeaconColorSensor.blue(){
                     
                 }
             }break;
             case "red":{

             }break;}
    }   
    
    }

    public void setPower(double FL,double FR, double BL, double BR){
        FrontLeft.setPower(FL);
        FrontRight.setPower(FR);
        BackLeft.setPower(BL);
        BackRight.setPower(BR);
    }

    public void SetMode(DcMotor.RunMode mode){
        FrontLeft.setMode(mode);
        FrontRight.setMode(mode);
        BackLeft.setMode(mode);
        BackRight.setMode(mode);
    }

    public double PidPowerAdjustment(int TargetAngle){

        float LinearCumulativeerror         = 0;
        float LinearproportionalCorrection;
        float LinearintegralCorrection;
        float LinearSlopeofderivitive;
        float LinearMaxCorrection           = 100;
        float LinearMinCorrection           = 15;
        float Linearerror                   = Math.abs(TargetAngle-getIntegratedZValue());
        LinearproportionalCorrection        = (LinearproportionalConstant*Linearerror);
        LinearCumulativeerror+=Linearerror;
        LinearintegralCorrection            = (LinearintegralConstant*LinearCumulativeerror);
        LinearSlopeofderivitive             = Linearerror-Linearlasterror;
        float Linearderivitivecorrection    = (LinearSlopeofderivitive*LinearderivitiveConstant);


        float LinearCorrection=LinearproportionalCorrection+LinearintegralCorrection+Linearderivitivecorrection;

        if (LinearCorrection>LinearMaxCorrection){LinearCorrection=LinearMaxCorrection;}

        else if (LinearCorrection<LinearMinCorrection){LinearCorrection=LinearMinCorrection;}

        else LinearCorrection=LinearCorrection;
        return LinearCorrection;

    }
}
