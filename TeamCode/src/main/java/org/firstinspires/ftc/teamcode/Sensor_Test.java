package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Trevor on 10/2/2016.
 */
@TeleOp(name="Sensor Test", group="6994 Bot")
public class Sensor_Test extends HardwareMap {
    Servo servo;
    ModernRoboticsI2cRangeSensor Range;
    GyroSensor Gyro;
    ColorSensor Color;
    OpticalDistanceSensor ODS;
    TouchSensor Ts;
    DcMotor motor;


    @Override
    public void init() {
        BackRightSideRangeSensor= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, backRightSideRangeSensor);
        Gyro=hardwareMap.gyroSensor.get(gyroSensor);
        Color=hardwareMap.colorSensor.get(beaconColorSensor);
        Gyro.calibrate();

    }

    @Override
    public void loop() {
        telemetry.addData("Range return",BackRightSideRangeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Gyro",Gyro.getHeading());
        telemetry.addData("Color Green",Color.green());
        telemetry.addData("Color Blue",Color.blue());
        telemetry.addData("Color Red",Color.red());






    }
}
