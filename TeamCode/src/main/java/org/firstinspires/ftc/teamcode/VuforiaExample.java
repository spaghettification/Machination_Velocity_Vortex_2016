package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by 70018 on 12/2/2016.
 */
public class BlueParametricButtonPusher extends LinearHardwareMap {
    @Override
    public void runOpMode(){
        AutonomousHardwareMap();
        InitializeServoPositions();
        StopAllMotors();
        Calibrate();
        Gyro.calibrate();
        double WhiteLineLocation = 60;
        double CoordianteTarget[] = {1,2};
        double OriginalPlacement = FrontRangeSensor.getDistance(DistanceUnit.INCH);
        double ParametricPathXTravel = OriginalPlacement - CoordianteTarget[0];
        double ParametricPathYTravel = WhiteLineLocation-CoordianteTarget[1];
        double HypotenuseOfParametricPath = Math.sqrt(Math.pow(ParametricPathXTravel,2)+Math.pow(ParametricPathYTravel,2));
        double FirstRotationTarget = Math.acos(ParametricPathXTravel/HypotenuseOfParametricPath);

        waitForStart();
        while (Gyro.isCalibrating()){}
        Turn(.25,0, (int) FirstRotationTarget,false);
        Drive(.5,HypotenuseOfParametricPath, (int) FirstRotationTarget,5,false);
        Turn(0,.25,0,false);
        FindWhiteLine(WhiteLineFinder,.25);
        //Button Push



    }
}
