package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoToZeroPointFive extends LinearOpMode {
    public static Servo servo1, servo2;
    public static double servoDesiredPos;
    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "ansh");
        servo2 = hardwareMap.get(Servo.class, "ayush");


        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                servo1.setPosition(0.5d);
                servo2.setPosition(0.5d);
            }

            if (gamepad1.b)
            {
                servo1.setPosition(1d);
                servo2.setPosition(0d);
            }

            if (gamepad1.x)
            {
                servo1.setPosition(1 - servoDesiredPos);
                servo2.setPosition(servoDesiredPos);
            }

            telemetry.addData("Servo 1 Position", servo1.getPosition());
            telemetry.addData("Servo 2 Position", servo2.getPosition());
            telemetry.update();
        }
    }
}
