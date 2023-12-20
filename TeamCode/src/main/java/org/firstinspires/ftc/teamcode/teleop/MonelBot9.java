package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.angle_pid.Drivetrain;
import org.firstinspires.ftc.teamcode.angle_pid.PIDConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@TeleOp(group = "Robot Main")
@Config
public class MonelBot9 extends LinearOpMode {
    SampleMecanumDrive drive = null;
    DriveTrain drivetrain = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;
    ElapsedTime inputTimer, outputTimer;
    public static double
            armServoOnePos, armServoTwoPos, wristServoPos, deliveryServoPos, armSliderServoPos;
    public static double
            gripperServoPos, intakeArmServoPos, intakeWristServoPos, crankServoPos;
    public static int levelZero = 0, levelOne = 200, levelTwo = 400, levelThree = 500;
    boolean
            armToggle = false, deliveryToggleOne = false, deliveryToggleTwo = false, intakeToggle = false, crankToggle = false, driveToggle = false, angleToggle1 = false, angleToggle2=false,stackFlag = false;;
    public static int intakeCounter, outtakeCounter,sliderCounter =0;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;

    public static double kp = 3.5, ki, kd = 1;
    double Kp = PIDConstants.Kp, Ki = PIDConstants.Ki, Kd = PIDConstants.Kd;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0, integralSum = 0;;
    private BHI260IMU imu;
    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_GRIP,
        INTAKE_RETRACT,
        INTAKE_INPUT,
        INTAKE_FINAL
    };
    public enum OuttakeState{
        OUTTAKE_START,
        OUTTAKE_PUSH,
        OUTTAKE_OPEN,
        OUTTAKE_OUTPUT,
        OUTTAKE_SLIDER,
        OUTTAKE_FINAL
    };
    MonelBot8.IntakeState inputState = MonelBot8.IntakeState.INTAKE_START;
    MonelBot8.OuttakeState outputState = MonelBot8.OuttakeState.OUTTAKE_START;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new DriveTrain(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drone =new Drone(hardwareMap, telemetry);

        inputTimer = new ElapsedTime();
        outputTimer = new ElapsedTime();
        timer = new ElapsedTime();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "analogInput");

        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        while (opModeInInit()){
            Arm.SetArmPosition(0.15,0.73);
            Intake.crankServo.setPosition(0.7);
            Intake.intakeArmServo.setPosition(0.5);
            Intake.intakeWristServo.setPosition(0.65);
            Drone.initialPos();
            Hanger.hangerServoOne.setPosition(0.25);
            Hanger.hangerServoTwo.setPosition(0.75);
            Intake.gripperServo.setPosition(1);
            inputTimer.reset();
            outputTimer.reset();
            intakeCounter = 0;
        }

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            if (currentGamepad1.start && !previousGamepad1.start) {
                imu.resetYaw();
            }
            double axonPosition = analogInput.getVoltage() / 3.3 * 360;

            // Main teleop loop goes here

            //drivetrain ---------------------------------------------------------------------------
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            DriveTrain.setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            double turn90 = angleWrap(Math.toRadians(90) - botHeading);
            double turn180 = angleWrap(Math.toRadians(180) - botHeading);
            if (currentGamepad1.left_trigger > 0.3 && !(previousGamepad1.left_trigger > 0.3)){
                drive.turn(turn90);
            }
            if (currentGamepad1.right_trigger > 0.3 && !(previousGamepad1.right_trigger > 0.3)){
                drive.turn(turn180);
            }
            drive.update();
            telemetry.addData("Bot heading", Math.toDegrees(botHeading));
            telemetry.addData("IntakeToggle", intakeToggle);
            telemetry.addData("sliderCounter", sliderCounter);
            telemetry.addData("stackFlag", stackFlag);
            telemetry.addData("IntakeCounter", intakeCounter);
            telemetry.addData("AXON Position", axonPosition);
            telemetry.addData("Beam Breaker State:", beamBreaker.getState());
            telemetry.addData("SliderMotorOne tick count", Slider.sliderMotorOne.getCurrentPosition());
            telemetry.addData("SliderMotorTwo tick count", Slider.sliderMotorTwo.getCurrentPosition());
            telemetry.addData("SliderMotorOne Current", Slider.sliderMotorOne.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("SliderMotorTwo Current", Slider.sliderMotorTwo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("HangerMotor tick count", Hanger.hangerMotor.getCurrentPosition());
            telemetry.addData("Hanger Current", Hanger.hangerMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("gripperServo", Intake.gripperServo.getPosition());
            telemetry.addData("intakeWristServo", Intake.intakeWristServo.getPosition());
            telemetry.addData("intakeArmServo", Intake.intakeArmServo.getPosition());
            telemetry.addData("crankServo", Intake.crankServo.getPosition());
            telemetry.addData("armServoOne", ArmV2.armServoOne.getPosition());
            telemetry.addData("armServoTwo", ArmV2.armServoOne.getPosition());
            telemetry.addData("wristServo", ArmV2.wristServo.getPosition());
            telemetry.addData("armSlider", ArmV2.armSliderServo.getPosition());
            telemetry.addData("deliveryServo", ArmV2.deliveryServo.getPosition());
            telemetry.addData("LeftFrontCurrent", DriveTrain.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightFrontCurrent", DriveTrain.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LeftRearCurrent", DriveTrain.leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightRearCurrent", DriveTrain.rightRear.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            drive.update();
        }
    }
    public double lifter_pid(double kp_lifter, double ki_lifter, double kd_lifter, int target)
    {
        lifter_posL = Slider.sliderMotorOne.getCurrentPosition();
        lifter_posR = Slider.sliderMotorTwo.getCurrentPosition();
        error_lifter = target - lifter_posL;
        error_diff = error_lifter - errorprev;
        error_int = error_lifter + errorprev;
        output_lifter = kp_lifter*error_lifter + kd_lifter*error_diff +ki_lifter*error_int;
        error_lifterR = target - lifter_posR;
        error_diffR = error_lifterR - errorprevR;
        error_intR = error_lifterR + errorprevR;
        output_lifterR = kp_lifter*error_lifterR + kd_lifter*error_diffR +ki_lifter*error_intR;
        errorprev = error_lifter;
        errorprevR = error_lifterR;
        return Math.abs(output_lifter);
    }
    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
