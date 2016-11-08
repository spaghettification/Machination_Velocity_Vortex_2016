package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Trevor on 10/5/2016.
 */
public class Falling_Bodies extends LinearOpMode {
    TouchSensor TopTS;
    TouchSensor BottomTS;
    Servo Gate;
    double starttime;
    double endtime;
    double initialvelocity;
    double finalvelocity;
    double accelerationduetogravity=9.81;
    double Totaltime=starttime-endtime;

    @Override
    public void runOpMode() throws InterruptedException {
        TopTS = hardwareMap.touchSensor.get("release");
        BottomTS = hardwareMap.touchSensor.get("stop");
        Gate = hardwareMap.servo.get("gate");
        waitForStart();
        while (opModeIsActive()) {
            if (TopTS.isPressed()) {
                wait(5000);
                Gate.setPosition(1);
                starttime = SystemClock.currentThreadTimeMillis();
            }
            if (BottomTS.isPressed()) {
                Gate.setPosition(0);
                endtime = SystemClock.currentThreadTimeMillis();
            }
            telemetry.addData("Start Time", starttime);
            telemetry.addData("EndTime", endtime);
            telemetry.addData("Final Velocity", calculatefinalvelocity(Totaltime,initialvelocity));
        }

    }
    public double calculatefinalvelocity(double totaltime,double initialvelocity  ){
        return initialvelocity-(accelerationduetogravity*totaltime);
    }

}


