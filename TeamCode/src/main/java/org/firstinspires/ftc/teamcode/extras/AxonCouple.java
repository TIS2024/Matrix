package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "ServoTest")
@Config
public class AxonCouple extends LinearOpMode {
    public Servo servo1 , servo2;
    public static double servoPos = 0.5
            ;
    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                servo1.setPosition(0.5);
                servo2.setPosition(0.5);
            }
            if (gamepad1.b){
                servo1.setPosition(1-servoPos);
                servo2.setPosition(servoPos);
            }
            telemetry.addData("ServoPosition", servo1.getPosition());
            telemetry.addData("ServoPosition", servo2.getPosition());
            telemetry.update();
        }
    }
}
