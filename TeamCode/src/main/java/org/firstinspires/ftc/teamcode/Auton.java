package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.PrintWriter;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;
/**CURRENT CONFIGURATION WORKS*/
/**
 * Created by Trevor on 10/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "EncoderTest2", group = "6994 Bot")

public class Auton extends LinearHardwareMap {
    int i;
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
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        sleep(200);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.125,0,1500,FirstPushOff,2);
        DriveWithEncoders(DriveSyle.ClockwiseWithGyroScope,.125, (int) Theta,0,0,2);
        DriveWithEncoders(DriveSyle.LinearWithEnocders,.125, (int) Theta,2000,Hypotenuse,4);
        DriveWithEncoders(DriveSyle.CounterClockwiseWithGyroScope,.125,0,2000,0,2);

    }
    public void initialize(){
        FrontLeft=hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight=hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft=hardwareMap.dcMotor.get(backLeftMotor);
        BackRight=hardwareMap.dcMotor.get(backRightMotor);
        Catapult = hardwareMap.dcMotor.get(catapult);
        CatapultStop = hardwareMap.touchSensor.get(catapultStop);

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


    enum DriveSyle{LinearWithEnocders, ClockwiseWithGyroScope,CounterClockwiseWithGyroScope}

    public void DriveWithEncoders(DriveSyle driveSyle, double Power,int TargetAngle, int MaxSpeed, double Distance, double Pause){
        int FrontLeftCurrent = FrontLeft.getCurrentPosition();
        int FrontRightCurrent = FrontRight.getCurrentPosition();
        int BackLeftCurrent = BackLeft.getCurrentPosition();
        int BackRightCurrent = BackRight.getCurrentPosition();

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
                    telemetry.addData("FrontLeft",FrontLeft.getCurrentPosition());
                    telemetry.addData("FrontRight",FrontRight.getCurrentPosition());
                    telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
                    telemetry.addData("BackRight",BackRight.getCurrentPosition());

                        telemetry.update();
                }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        }break;

            case ClockwiseWithGyroScope:{
                if (opModeIsActive()){

                setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    runtime.reset();
                while (Math.abs(getIntegratedZValue()-TargetAngle)<1.2&&runtime.seconds()>Pause){
                    FrontLeft.setPower(Power);
                    FrontRight.setPower(-Power);
                    BackLeft.setPower(Power);
                    BackRight.setPower(-Power);
                }

                }
            }break;

        case CounterClockwiseWithGyroScope:{
            if (opModeIsActive()){

                setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                runtime.reset();
                while (Math.abs(getIntegratedZValue()-TargetAngle)<1.2&&runtime.seconds()>Pause){
                    FrontLeft.setPower(-Power);
                    FrontRight.setPower(Power);
                    BackLeft.setPower(-Power);
                    BackRight.setPower(Power);
                }

            }
        }break;
    }
    }
}
