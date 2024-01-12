package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Config
public class BeamBreaker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("BeamBreaker", beamBreaker.getState());
            telemetry.update();
        }
    }
}
