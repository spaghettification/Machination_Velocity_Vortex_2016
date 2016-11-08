package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Trevor on 9/19/2016.
 */
public class Kaintest extends OpMode {
    DcMotor motor;
    @Override
    public void init() {
        motor=hardwareMap.dcMotor.get("m");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setTargetPosition(1440);

        motor.setPower(1);

    }
}
