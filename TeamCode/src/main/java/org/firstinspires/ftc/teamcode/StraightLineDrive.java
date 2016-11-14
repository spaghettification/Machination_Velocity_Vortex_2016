package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Trevor on 10/16/2016.
 */
@Autonomous(name = "LineDrive", group = "6994")@Disabled
public class StraightLineDrive extends FTC_6994_Template {
    public final PathSeg[] GyroTest = {new PathSeg(.25, 0, 24, FTC_6994_Template.DriveStyle.Linear)};
    public final PathSeg[] Return = {new PathSeg(.25, 180, 0, DriveStyle.Clockwise), new PathSeg(.25, 180, 24, DriveStyle.Linear), new PathSeg(.25, 0, 0, DriveStyle.Clockwise),};
    double Old_Theta;
    boolean DriftingRight;
    boolean CorrectValueReached;
    int i;

    public void init() {
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        Gyro = hardwareMap.gyroSensor.get("Gyro");
        Gyro.calibrate();
        resetDriveEncoders();
    }

    int j;


    public void loop() {
        while (i < 30) {
            switch (j) {
                case 1: {
                    startPath(GyroTest);//Drive Forward


                    if (pathComplete()) {//wait until the path is complete
                        if (Old_Theta == getIntegratedZValue()) {//if The robot already drives straight then make no adjustment
                            CorrectValueReached = true;
                            telemetry.addData("CorrectValueReached", CorrectValueReached);
                            telemetry.addData("ThetaPowerConstant", ThetaPowerConstant);
                            i = 40;
                        }
                    }
                    if (i == 1) {//On the first Loop
                        if (getIntegratedZValue() > 0) {// Describe the problem
                            DriftingRight = true;
                        } else if (getIntegratedZValue() < 0) {
                            DriftingRight = false;
                        } else {
                            CorrectValueReached = true;
                        }
                    } else {
                        if (DriftingRight) {
                            if (Old_Theta > getIntegratedZValue()) {//in this case raise the adjustment level
                                ThetaPowerConstant = +.1;
                                j++;
                            }

                        }
                        if (!DriftingRight) {//in this case lower the adjustment level
                            if (Old_Theta > getIntegratedZValue()) {
                                ThetaPowerConstant = -.1;
                                j++;
                            }
                        }
                    }


                }
                break;
                case 2: {
                    startPath(Return);
                    if (pathComplete()) {
                        i++;
                        j--;
                    }
                    //autonomously repeat until robot drives straight
                }
            }


        }

    }
}


