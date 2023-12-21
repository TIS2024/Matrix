package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;

@TeleOp(group = "Robot Main")
@Config
public class MonelBotTest extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;
    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    double armServoPos, wristServoPos, deliveryServoPos, intakeArmServoPos, intakeWristServoPos, gripperServoPos, crankServoPos, armSliderServoPos;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drone =new Drone(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            // Main teleop loop goes here

            //drivetrain ---------------------------------------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad2.left_stick_y, -1, 1), 3),
                    Math.pow(Range.clip(gamepad2.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad2.right_stick_x * HEADING)
            );
            drive.update();
            telemetry.addData("heading", poseEstimate.getHeading());

            //Slider
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                telemetry.addLine("DPad_UP_Pressed");
                Slider.IncreaseExtension(200);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                telemetry.addLine("DPad_DOWN_Pressed");
                Slider.DecreaseExtension(0);
            }

            telemetry.addData("SliderMotorOne tick count", Slider.sliderMotorOne.getCurrentPosition());
            telemetry.addData("SliderMotorTwo tick count", Slider.sliderMotorTwo.getCurrentPosition());
            telemetry.addData("SliderMotorOne Current", Slider.sliderMotorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SliderMotorTwo Current", Slider.sliderMotorTwo.getCurrent(CurrentUnit.AMPS));
            //--------------------------------------------------------------------------------------

            //Arm
            if (currentGamepad1.x && !previousGamepad1.x){
                ArmV2.SetArmPosition(armServoPos,wristServoPos);
            }
            if(currentGamepad1.start && !previousGamepad1.start){
                ArmV2.SliderLink(armSliderServoPos);
            }
            //Intake
            if(currentGamepad1.y && !previousGamepad1.y) {
                Intake.SetArmPosition(intakeArmServoPos, intakeWristServoPos);
            }
            //Gripper
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                Intake.IntakePixel(gripperServoPos);
            }
            //Crank
            if (currentGamepad1.b && !previousGamepad1.b){
                Intake.CrankPosition(crankServoPos);
            }
            //Delivery
            if(currentGamepad1.a && !previousGamepad1.a){
                Arm.DropPixel(deliveryServoPos);
            }
            //Hanger
            if(currentGamepad1.back && !previousGamepad1.back){
                Hanger.ExtendHanger();
            }
            if (currentGamepad1.dpad_up){
                Hanger.LiftRobot();
            }
            if (currentGamepad1.dpad_down){
                Hanger.PutDownRobot();
            }
        }
    }
}
