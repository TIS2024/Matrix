package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class DistanceSensorTest extends LinearOpMode {
    AnalogInput distanceSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(AnalogInput.class, "distanceSensor");
        waitForStart();
        while (opModeIsActive()){
            double voltage = distanceSensor.getVoltage();

            telemetry.addData("Voltage", voltage);
            telemetry.addData("distance", getDistance());
            telemetry.addData("distance3", getdist(distanceSensor.getVoltage()));
            telemetry.addData("distance2", convertToInches(distanceSensor.getVoltage()));
            telemetry.update();
        }
    }
    public double getDistance()
    {
        return (Math.pow(distanceSensor.getVoltage(), -1.2045)) * 27.726;
    }
    private static final double VOLTAGE_AT_NO_DISTANCE = 1.6; // Replace with your sensor's value
    private static final double VOLTAGE_AT_MAX_DISTANCE = 0.5; // Replace with your sensor's value
    private static final double MAX_DISTANCE_INCHES = 10.0; // Replace with your sensor's value
    public static double convertToInches(double analogVoltage) {
        // Calculate the voltage range
        double voltageRange = VOLTAGE_AT_NO_DISTANCE - VOLTAGE_AT_MAX_DISTANCE;

        // Calculate the distance range
        double distanceRange = MAX_DISTANCE_INCHES;

        // Calculate the distance using the formula: distance = maxDistance - ((voltage - minVoltage) / voltageRange) * distanceRange
        double distance = MAX_DISTANCE_INCHES - ((analogVoltage - VOLTAGE_AT_MAX_DISTANCE) / voltageRange) * distanceRange;

        // Ensure distance is within valid range
        distance = Math.max(0, Math.min(MAX_DISTANCE_INCHES, distance));

        return distance;
    }
    public double getdist(double voltage){
        double dist = 2.076/(voltage-0.011);
        return dist;
    }
}
