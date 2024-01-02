package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class LEDTest extends LinearOpMode {
    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    @Override
    public void runOpMode() throws InterruptedException {
        // Get the LED colors and touch sensor from the hardwaremap
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");

        // Wait for the play button to be pressed
        waitForStart();
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        while(opModeIsActive()){
            if (gamepad1.x){
                //Touch Sensor is not pressed
                greenLED.setState(true);
                redLED.setState(true);

            } else {
                //Touch Sensor is pressed
                redLED.setState(false);
                greenLED.setState(false);
            }
        }
    }
}
