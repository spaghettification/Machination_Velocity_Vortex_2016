/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

*/
/**
 * Created by Trevor on 11/6/2016.
 *//*


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Shooter", group = "6994 Bot")@Disabled
public class Shooter extends LinearHardwareMap {

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft=hardwareMap.dcMotor.get(frontLeftMotor);
        FrontRight=hardwareMap.dcMotor.get(frontRightMotor);
        BackLeft=hardwareMap.dcMotor.get(backLeftMotor);
        BackRight=hardwareMap.dcMotor.get(backRightMotor);
        Catapult=hardwareMap.dcMotor.get(catapult);
        BallControl=hardwareMap.servo.get(ballControll);
        CatapultStop=hardwareMap.touchSensor.get(catapultStop);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BallControl.setPosition(ballControlEngagedPosition);
        waitForStart();
        FrontLeft.setPower(.125);
        FrontRight.setPower(.125);
        BackLeft.setPower(.125);
        BackRight.setPower(.125);
        sleep(1000);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        Catapult.setPower(1);
        sleep(8000);
        if (CatapultStop.isPressed()){
            Catapult.setPower(0);
            BallControl.setPosition(ballControlStartPosition);
        }
        sleep(300);
        Catapult.setPower(1);
        sleep(8000);
       //BallControl.setPosition(ballControlEngagedPosition);

    }
}
*/
