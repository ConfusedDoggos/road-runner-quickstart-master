package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Config
@TeleOp(name = "SensorTesting", group = "TeleOp")
public class SensorTesting extends LinearOpMode {
    DistanceSensor distSensor;
    ColorSensor colorSensor;
    double distance;

    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // If the distance in centimeters is less than 10, set the power to 0.3
            distance = distSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("DistanceSensed", distance);
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
    }
}