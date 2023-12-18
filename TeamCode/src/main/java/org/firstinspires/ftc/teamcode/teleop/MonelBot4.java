package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "Robot Main")
@Config
public class MonelBot4 extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Arm arm = null;
    Hanger hanger = null;
    Intake intake = null;
    Drone drone = null;
    TwoWheelTrackingLocalizer twtl = null;
    ElapsedTime inputTimer, outputTimer;

    public DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    public static double
            armServoPos, wristServoPos, deliveryServoPos;
    public static int levelZero = 0, levelOne = 250;
    public static double
            gripperServoPos, intakeArmServoPos, intakeWristServoPos, crankServoPos;
    boolean
            armToggle = false, deliveryToggleOne = false, deliveryToggleTwo = false, intakeToggle = false, crankToggle = false, driveToggle = false;
    public static int intakeCounter;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target;
    public static double kp = 3.5, ki, kd = 1;

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
        OUTTAKE_FINAL
    };
    IntakeState inputState = IntakeState.INTAKE_START;
    OuttakeState outputState = OuttakeState.OUTTAKE_START;
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        drone =new Drone(hardwareMap, telemetry);
        twtl = new TwoWheelTrackingLocalizer(hardwareMap, drive);

        inputTimer = new ElapsedTime();
        outputTimer = new ElapsedTime();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);

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
            Hanger.hangerServo.setPosition(0.3);
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

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
//                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
//            );
//            telemetry.addData("heading", poseEstimate.getHeading());
//            drive.update();
            //--------------------------------------------------------------------------------------

            switch (inputState){
                case INTAKE_START:
                    //waiting for input
                    if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper && (intakeCounter == 0)){
                        Intake.intakeArmServo.setPosition(0.4); Intake.intakeWristServo.setPosition(0.485);
                        Intake.IntakePixel(0.9);
                        Arm.armServo.setPosition(0.3);Arm.wristServo.setPosition(0.735);
                        Arm.DropPixel(1);
                        inputTimer.reset();
                        inputState = IntakeState.INTAKE_EXTEND;
                    }
                    break;
                case INTAKE_EXTEND:
                    Intake.CrankPosition(0.5);
                    if (inputTimer.milliseconds() >= 200){
                        inputState = IntakeState.INTAKE_GRIP;
                        inputTimer.reset();
                    }
                    break;
                case INTAKE_GRIP:
                    if (!beamBreaker.getState()){
                        Intake.intakeArmServo.setPosition(0.4); Intake.intakeWristServo.setPosition(0.45);
                        TrajectorySequence IntakePixel = drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(()->{Intake.CrankPosition(0.38);})
                                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.IntakePixel(0.75);})
                                .waitSeconds(0.3)
                                .build();
                        drive.followTrajectorySequence(IntakePixel);
                        drive.update();
                        if (inputTimer.milliseconds() >= 500){ //800
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_RETRACT;
                        }
                    }
                    if (beamBreaker.getState() && inputTimer.milliseconds()>=7000){
                        TrajectorySequence CancelIntakePixel = drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5); Intake.intakeWristServo.setPosition(0.65);})
                                .waitSeconds(0.2)
                                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                                .waitSeconds(0.3)
                                .addTemporalMarker(()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.735);})
                                .waitSeconds(0.3)
                                .build();
                        drive.followTrajectorySequence(CancelIntakePixel);
                        drive.update();
                        intakeCounter = 0;
                        if (inputTimer.milliseconds()>8000){
                            inputState = IntakeState.INTAKE_START;
                        }
                    }
                    break;
                case INTAKE_RETRACT:
                    Intake.intakeArmServo.setPosition(0.4);
                    Intake.intakeWristServo.setPosition(0.5);
                    Intake.CrankPosition(0.69);
                    if (inputTimer.milliseconds() >= 500){ //800
                        inputState = IntakeState.INTAKE_INPUT;
                        inputTimer.reset();
                    }
                    break;
                case INTAKE_INPUT:
                    if (inputTimer.milliseconds() >= 200){
                        Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);
                        if(inputTimer.milliseconds() >= 500){ //600
                            Intake.intakeArmServo.setPosition(0.79);
                            if(axonPosition <= 130){ //inputTimer.milliseconds() >= 900 &&
                                Intake.intakeWristServo.setPosition(0.45);Intake.intakeArmServo.setPosition(1);Intake.crankServo.setPosition(0.7);
                                inputState = IntakeState.INTAKE_FINAL;
                                inputTimer.reset();
                            }
                        }
                    }
                    break;
                case INTAKE_FINAL:
                    if (inputTimer.milliseconds() >= 200){
                        Arm.wristServo.setPosition(0.735);Arm.armServo.setPosition(0.15);
                        if (inputTimer.milliseconds() >= 400){
                            Arm.DropPixel(0.45);
                            Arm.armServo.setPosition(0);
                            output_power = lifter_pid(kp, ki, kd, -10);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(-10, output_power);
                            if (inputTimer.milliseconds() >= 500){ //600
                                output_power = lifter_pid(kp, ki, kd, 0);
                                if (output_power > 0.9) {
                                    output_power = 1;
                                } else if (output_power < 0.2) {
                                    output_power = 0;
                                }
                                slider.extendTo(0, output_power);
                                Arm.armServo.setPosition(0.15);
                                intakeCounter = 1;
                                inputState = IntakeState.INTAKE_START;
                            }
                        }
                    }
                    break;
                default:
                    inputState = IntakeState.INTAKE_START;
                    intakeCounter = 0;
            }

            switch (outputState){
                case OUTTAKE_START:
                    //waiting for input
                    if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                        outputState = OuttakeState.OUTTAKE_PUSH;
                        outputTimer.reset();
                    }
                    break;
                case OUTTAKE_PUSH:
                    Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.crankServo.setPosition(0.7);
                    Arm.wristServo.setPosition(0.735);Arm.armServo.setPosition(0.15);
                    if (outputTimer.milliseconds() >= 200){
                        Arm.DropPixel(0.45);
                        Arm.armServo.setPosition(0);
                        output_power = lifter_pid(kp, ki, kd, -10);
                        if (output_power > 0.9) {
                            output_power = 1;
                        } else if (output_power < 0.2) {
                            output_power = 0;
                        }
                        slider.extendTo(-10, output_power);
                        if (outputTimer.milliseconds() >= 400){
                            output_power = lifter_pid(kp, ki, kd, 0);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(0, output_power);
                            Arm.armServo.setPosition(0.15);
                            outputTimer.reset();
                            outputState = OuttakeState.OUTTAKE_OPEN;
                        }
                    }
                    break;
                case OUTTAKE_OPEN:
                    Intake.IntakePixel(1);
                    if (outputTimer.milliseconds() >= 100){
                        Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.50);
                        if(outputTimer.milliseconds() >= 400){
                            Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.65);
                            if (outputTimer.milliseconds() >= 600){
                                Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.65);
                                outputTimer.reset();
                                outputState = OuttakeState.OUTTAKE_OUTPUT;
                            }
                        }
                    }
                    break;
                case OUTTAKE_OUTPUT:
                    Arm.armServo.setPosition(0.5);Arm.wristServo.setPosition(0.175);
                    if (outputTimer.milliseconds() >= 200){
                        outputTimer.reset();
                        outputState = OuttakeState.OUTTAKE_FINAL;
                    }
                    break;
                case OUTTAKE_FINAL:
                    Intake.crankServo.setPosition(0.7);
                    Intake.intakeArmServo.setPosition(0.5);
                    Intake.intakeWristServo.setPosition(0.65);
                    if (outputTimer.milliseconds()>=100){
                        outputTimer.reset();
                        intakeCounter = 0;
                        outputState = OuttakeState.OUTTAKE_START;
                    }
                    break;
                default:
                    outputState = OuttakeState.OUTTAKE_START;
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && (intakeCounter == 1) && (Intake.intakeArmServo.getPosition() == 1)){
                intakeCounter = 0;
                TrajectorySequence ResetIntake = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Arm.DropPixel(0.75);})
                        .addTemporalMarker(()->{Arm.armServo.setPosition(0.3);Arm.wristServo.setPosition(0.735);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.50);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.65);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.65);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.735);})
                        .build();
                drive.followTrajectorySequenceAsync(ResetIntake);
                drive.update();
            }

            if(currentGamepad1.y && !previousGamepad1.y && (inputState!=IntakeState.INTAKE_START || outputState!=OuttakeState.OUTTAKE_START)){
                inputState = IntakeState.INTAKE_START;
                outputState = OuttakeState.OUTTAKE_START;
                TrajectorySequence ResetRobot = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Intake.CrankPosition(0.38);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65); Intake.intakeArmServo.setPosition(0.5);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.73);Arm.armServo.setPosition(0.15);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Arm.DropPixel(1);})
                        .waitSeconds(0.5)
                        .addTemporalMarker(()->{})
                        .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.65); Intake.intakeArmServo.setPosition(0.5);})
                        .addTemporalMarker(()->{Intake.IntakePixel(1);})
                        .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Arm.armServo.setPosition(0.15);Arm.wristServo.setPosition(0.73);})
                        .waitSeconds(0.5)
                        .build();
                drive.followTrajectorySequenceAsync(ResetRobot);
                drive.update();
            }

            if(currentGamepad1.b && !previousGamepad1.b){
                //drop 1st pixel
                deliveryServoPos = 0.75;
                Arm.DropPixel(deliveryServoPos);
                TrajectorySequence DropPixelOne = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Arm.DropPixel(0.75);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.2);Arm.armServo.setPosition(0.48);})
                        .waitSeconds(0.2)
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.2);Arm.armServo.setPosition(0.5);})
                        .build();
                drive.followTrajectorySequenceAsync(DropPixelOne);
                drive.update();
            }
            if(currentGamepad1.a && !previousGamepad1.a){
                //drop 2nd pixel
                output_power = lifter_pid(kp, ki, kd, levelZero);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                TrajectorySequence DropPixelTwo = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Arm.DropPixel(1);})
                        .waitSeconds(0.3)
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.5);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Arm.wristServo.setPosition(0.73);Arm.armServo.setPosition(0.15);})
                        .addTemporalMarker(()->{slider.extendTo(levelZero, output_power);})
                        .waitSeconds(0.2)
                        .build();
                drive.followTrajectorySequenceAsync(DropPixelTwo);
                drive.update();
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                output_power = lifter_pid(kp, ki, kd, levelOne);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                slider.extendTo(levelOne, output_power);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                output_power = lifter_pid(kp, ki, kd, levelZero);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                slider.extendTo(levelZero, output_power);
            }
            if (currentGamepad1.x && !previousGamepad1.x){
                TrajectorySequence endGame = drive.trajectorySequenceBuilder(startPose)
                        .addTemporalMarker(()->{Drone.droneServo.setPosition(0.3);})
                        .waitSeconds(0.1)
                        .addTemporalMarker(()->{Hanger.hangerServo.setPosition(0.1);})
                        .waitSeconds(0.5)
                        .addTemporalMarker(()->{Hanger.hangerServo.setPosition(0.0);})
                        .waitSeconds(0.3)
                        .build();
                drive.followTrajectorySequenceAsync(endGame);
                drive.update();
            }
            if (currentGamepad1.dpad_right){
                Hanger.LiftRobot();
            }
            if (currentGamepad1.dpad_left){
                Hanger.PutDownRobot();
            }
//            if(currentGamepad1.right_trigger>0.3){
//                THROTTLE = 0.3;
//                HEADING = 0.3;
//                TURN = 0.3;
////                driveToggle = !driveToggle;
//            }
//            else {
//                THROTTLE = 1;
//                HEADING = 1;
//                TURN = 1;
//            }

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
                levelOne += 50;
            }
            if(currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
                levelOne -= 50;
            }
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){
                Arm.DropPixel(0.75);
            }
            if(currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
                Arm.DropPixel(1);
            }
            if(currentGamepad2.a && !previousGamepad2.a){
                armServoPos = 0.6;
                wristServoPos = 0.15;
                Arm.SetArmPosition(armServoPos, wristServoPos);
            }
            if(currentGamepad2.b && !previousGamepad2.b){
                armServoPos = 0.5;
                wristServoPos = 0.175;
                Arm.SetArmPosition(armServoPos, wristServoPos);
            }
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                gripperServoPos = 0.75;
                Intake.IntakePixel(gripperServoPos);
            }
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                gripperServoPos = 1;
                Intake.IntakePixel(gripperServoPos);
            }
            if(currentGamepad2.x && previousGamepad2.x){
                Intake.SetArmPosition(intakeArmServoPos, intakeWristServoPos);
            }
            if(currentGamepad2.y && previousGamepad2.y){
                Arm.SetArmPosition(armServoPos, wristServoPos);
            }
            if (currentGamepad2.left_trigger>0.1 && !(previousGamepad2.left_trigger >0.1)){
                Intake.crankServo.setPosition(0.38);
            }
            if (currentGamepad2.right_trigger>0.1 && !(previousGamepad2.right_trigger >0.1)){
                Intake.crankServo.setPosition(0.69);
            }

            telemetry.addData("IntakeCounter", intakeCounter);
            telemetry.addData("Bot Heading", botHeading);
            telemetry.addData("ParallelEnc Counts", twtl.parallelEncoder.getCurrentPosition());
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
            telemetry.addData("armServo", Arm.armServo.getPosition());
            telemetry.addData("wristServo", Arm.wristServo.getPosition());
            telemetry.addData("deliveryServo", Arm.deliveryServo.getPosition());
//            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
//            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
//            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
//            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            telemetry.addData("LeftFrontVelocity", leftFront.getVelocity());
            telemetry.addData("RightFrontVelocity", rightFront.getVelocity());
            telemetry.addData("LeftRearVelocity", leftRear.getVelocity());
            telemetry.addData("RightRearVelocity", rightRear.getVelocity());
            telemetry.addData("LeftFrontCurrent", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightFrontCurrent", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LeftRearCurrent", leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RightRearCurrent", rightRear.getCurrent(CurrentUnit.AMPS));
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
}