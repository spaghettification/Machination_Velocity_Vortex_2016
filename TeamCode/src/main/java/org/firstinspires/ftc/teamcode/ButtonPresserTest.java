package org.firstinspires.ftc.teamcode;

/**
 * Created by Trevor on 11/26/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ButtonPresserTest", group = "6994 Bot")
public class ButtonPresserTest extends LinearHardwareMap {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousHardwareMap();
        StopAllMotors();
        //InitializeServoPositions();
        waitForStart();

        while(opModeIsActive()){
            for (int i = 0;i<20;i++){
                ButtonPusherArm.setPosition(i/20);/*
                for (int j = 0;j<20;j++){
                    ButtonPusherArm.setPosition(j/20);
                sleep(3000);
                }*/
            telemetry.addData("buttonPusher", ButtonPusher.getPosition());
            telemetry.addData("Arm", ButtonPusherArm.getPosition());
            telemetry.update();
            sleep(3000);}
        }
    }
}
