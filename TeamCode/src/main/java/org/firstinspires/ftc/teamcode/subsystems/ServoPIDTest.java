package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoPIDTest extends LinearOpMode {
    Servo servo;
    public static double servoPos;
    public static double Kp;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while(opModeIsActive()){
            // Set the desired servo position
            double targetPosition = servoPos; // Replace with your desired position between 0 and 1

            // Use proportional control to smoothly move the servo to the target position
            while (opModeIsActive() && !isTargetPositionReached(servo.getPosition(), targetPosition)) {
                double currentPosition = servo.getPosition();
                double error = targetPosition - currentPosition;

                // Proportional control formula
                double power = error * Kp; // Adjust the proportional gain as needed

                // Apply the power to the servo
                servo.setPosition(currentPosition + power);

                // Optionally, you can add a small delay to avoid rapid changes
                sleep(10);
            }
        }
    }
    public static boolean isTargetPositionReached(double currentPosition, double targetPosition){
        double tolerance = 0.001; // Adjust the tolerance as needed
        return Math.abs(currentPosition - targetPosition) < tolerance;
    }
}
