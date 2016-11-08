package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Trevor on 10/28/2016.
 */
public class trial1 extends LinearOpMode {
    DcMotor FrontLeft;
    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft=hardwareMap.dcMotor.get("FL");
        waitForStart();

       Thread.sleep(1000);
        FrontLeft.setPower(.125);
        FrontLeft.setTargetPosition(10000);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (FrontLeft.isBusy()){
            telemetry.addData("FrontLeft",FrontLeft.isBusy());
        } else{FrontLeft.setPower(0);}


    }
}
