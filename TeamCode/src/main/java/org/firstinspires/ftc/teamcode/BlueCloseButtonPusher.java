package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 11/21/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueCloseButtonPusher", group = "6994 Bot")
public class BlueCloseButtonPusher extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        AutonomousHardwareMap();
        StopAllMotors();
        InitializeServoPositions();
        Calibrate();

        waitForStart();while(opModeIsActive()) {
            while (Gyro.isCalibrating()) {
                idle();
            }
            Drive(.375, 8, 0, 5, false);
            // shootParticle(1);
            Turn(.125, 38, false, "clockwise");
            Drive(.375, 60,getIntegratedZValue(),4, false);
            sleep(1000);
            FindWhiteLine(WhiteLineFinder,.175);
            Turn(.125, 90, true, "clockwise");
            ButtonPush("Blue");
            setPower(0, 0, 0, 0);
            Drive(.325, -8, 90, 2, false);
            Turn(.125, 0, false, "CounterClockwise");
            Drive(.25, 40, getIntegratedZValue(), 5, false);
            FindWhiteLine(WhiteLineFinder, .175);
            Turn(.125, 90, false, "Clockwise");
            ButtonPush("Blue");
            sleep(500);
            Drive(.125, -6, 90, 3, false);
            Turn(.125, 0, false, "CounterClockwise");
            sleep(500);
            Drive(.125, -8, getIntegratedZValue(), 5, false);
            Turn(.125, 150, false, "clockwise");
            //l shootParticle(1);
            Drive(.25, 50, getIntegratedZValue(), 5, false);


        }
    }

}

