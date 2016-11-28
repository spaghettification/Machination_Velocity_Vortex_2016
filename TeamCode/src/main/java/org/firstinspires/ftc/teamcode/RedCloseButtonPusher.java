package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 11/21/2016.
 */
public class RedCloseButtonPusher extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousHardwareMap();
        StopAllMotors();
        InitializeServoPositions();
        Calibrate();

        waitForStart();

        /*Drive(.25, 8, 0, 5, false);
        shootParticle(1);
        Turn(.125, -51, false);
        Drive(.25, 66.843, 0, 4, false);
        FindWhiteLine(WhiteLineFinder,.25);
        Turn(.125, 90, true);
        ButtonPush("Red");
        setPower(0, 0, 0, 0);
        Drive(.125,-8,90,2,false);
        Turn(.125,0,false);
        Drive(.25,48,getIntegratedZValue(),5,false);
        FindWhiteLine(WhiteLineFinder,.25);
        Turn(.125,90,true);
        ButtonPush("Red");
        Drive(.125, -6, 90, 3, false);
        Turn(.125, 150, false);
        shootParticle(1);
        Drive(.25,30,getIntegratedZValue(),4,false);
        StopAllMotors();
        InitializeServoPositions();
        requestOpModeStop();
*/


    }
}
