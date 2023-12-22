package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "ServoTest")
@Config
public class ServoTest extends LinearOpMode {
    public Servo servo,servo1,servo2,servo3,servo4,servo5;
    public static double servoPos;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                servo.setPosition(0);
                servo1.setPosition(0);
                servo2.setPosition(0);
                servo3.setPosition(0);
                servo4.setPosition(0);
                servo5.setPosition(0);

            }
            if (gamepad1.b){
                servo.setPosition(0.5);
                servo1.setPosition(0.5);
                servo2.setPosition(0.5);
                servo3.setPosition(0.5);
                servo4.setPosition(0.5);
                servo5.setPosition(0.5);

            }
            if (gamepad1.x){
                servo.setPosition(1);
                servo1.setPosition(1);
                servo2.setPosition(1);
                servo3.setPosition(1);
                servo4.setPosition(1);
                servo5.setPosition(1);



            }
            if (gamepad1.y){
                servo.setPosition(servoPos);
            }
            telemetry.addData("ServoPosition", servo.getPosition());
            telemetry.update();
        }
    }
}
