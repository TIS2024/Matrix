package org.firstinspires.ftc.teamcode.auto.new_autos;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.AsyncFollowingFSM;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="BlueFarAutoTest", group = "Cycle_Autos")
@Config
@Disabled
public class BlueFarAutoTest extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
    private static final String[] LABELS = {
            "beacon"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    public static int val=0;
    double x;
    double y;
    String propPosition = " ";

    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;

    public static double kp = 4, ki, kd = 1.7;
    public static Pose2d PurpleRightPos = new Pose2d(-35,32, -Math.PI), YellowRightPos, StackRightPos = new Pose2d(-51 , 12-0.5, -Math.PI);
    public static Vector2d PurpleRight, YellowRight = new Vector2d(53.5,30.5), StackRight = new Vector2d(-51, 12.5);


    public static Pose2d PurpleLeftPos = new Pose2d(-44,30, 0), YellowLeftPos, StackLeftPos = new Pose2d(-51 , 12, -Math.PI);
    public static Vector2d PurpleLeft, YellowLeft = new Vector2d(51,48), StackLeft = new Vector2d(-51, 12);


    public static Pose2d PurpleCenterPos = new Pose2d(-51,24, 0), YellowCenterPos, StackCenterPos = new Pose2d(-51 , 12, -Math.PI);
    public static Vector2d PurpleCenter, YellowCenter = new Vector2d(51,38), StackCenter = new Vector2d(-51, 12);

    enum AutoTrajectoryRight {
        Start,
        AutoTrajectoryRightPurple,
        CenterPathPlacing,
        AutoTrajectoryRightYellow,
        CenterPathPicking,
        CenterPathPlacing2,
        AutoTrajectoryRightYellow2,
        ParkingOut,
        IDLE
    }
    enum AutoTrajectoryCenter {
        Start,
        AutoTrajectoryCenterPurple,
        CenterPathPlacing,
        AutoTrajectoryCenterYellow,
        CenterPathPicking,
        CenterPathPlacing2,
        AutoTrajectoryCenterYellow2,
        ParkingOut,
        IDLE
    }
    enum AutoTrajectoryLeft {
        Start,
        AutoTrajectoryLeftPurple,
        CenterPathPlacing,
        AutoTrajectoryLeftYellow,
        CenterPathPicking,
        CenterPathPlacing2,
        AutoTrajectoryLeftYellow2,
        ParkingOut,
        IDLE
    }
    AutoTrajectoryRight currentState = AutoTrajectoryRight.Start;
    AutoTrajectoryCenter currentState2 = AutoTrajectoryCenter.Start;
    AutoTrajectoryLeft currentState3 = AutoTrajectoryLeft.Start;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(-39, 64, 0);
        drive.setPoseEstimate(startPose);
        initTfod();


        TrajectorySequence AutoTrajectoryRightPurple = drive.trajectorySequenceBuilder(startPose)
                // right line
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})
                .lineToSplineHeading(PurpleRightPos)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
                .build();

        TrajectorySequence CenterPathPlacing = drive.trajectorySequenceBuilder(AutoTrajectoryRightPurple.end())
                .lineToSplineHeading(new Pose2d(-34 , 12, -Math.PI))
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + 0.01);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.5);})
                .lineToSplineHeading(StackRightPos)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
//                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-34,12),0)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .waitSeconds(0.2)//0.3
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .waitSeconds(0.2)//0.4
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})
                .waitSeconds(0.1)//0.3
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
//                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
//                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(0.15, 0.165);})
//                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})
                .splineToConstantHeading(new Vector2d(36,12),0) //28
                .build();

        TrajectorySequence AutoTrajectoryRightYellow = drive.trajectorySequenceBuilder(CenterPathPlacing.end())
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
                .lineToConstantHeading(new Vector2d(53.5 + 1, 40))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.54, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.54, 0.68);})
//                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.68);})
                .lineToConstantHeading(YellowRight)
                .addTemporalMarker(()->{arm.setArmPos(0.53,0.68);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.54, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
//                .waitSeconds(5)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking = drive.trajectorySequenceBuilder(AutoTrajectoryRightYellow.end())
                //round1
                .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33, 12), -Math.PI)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Ma.0th.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .lineToConstantHeading(StackRight)
//                .resetConstraints()
//                .setReversed(true)
                .build();

        TrajectorySequence CenterPathPlacing2 = drive.trajectorySequenceBuilder(CenterPathPicking.end())
//                .lineToSplineHeading(new Pose2d(-34 , 12, -Math.PI))
                .lineToConstantHeading(new Vector2d(-34, 12))
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.55);Intake.intakeWristServo.setPosition(0.37);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.5);})
                .lineToSplineHeading(StackRightPos)
//                .lineToSplineHeading(new Pose2d(-51 , 11.5, -Math.PI))
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.55, 0.4);})
//                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-34,12),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})
//                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(0.15, 0.165);})
//                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})
//                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);Intake.IntakePixel(0.77);})
//                .waitSeconds(0.2)//0.3
//                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.75);Intake.IntakePixel(0.77);})
//                .waitSeconds(0.2)//0.4
//                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);Intake.IntakePixel(0.77);})
//                .waitSeconds(0.1)//0.3
//                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.165);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .splineToConstantHeading(new Vector2d(36,12),0) //28
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.165);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.155);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(0, 1);})
                .build();

        TrajectorySequence AutoTrajectoryRightYellow2 = drive.trajectorySequenceBuilder(CenterPathPlacing2.end())
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(30))
//                .lineToConstantHeading(new Vector2d(54, 40))
                .splineToConstantHeading(new Vector2d(53.5, 40), 0)
                .lineToConstantHeading(new Vector2d(54, 40))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.53, 0.175);}) //-1
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.53, 0.68);}) //-0.4
//                .waitSeconds(0.2)
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{ArmV2.DropPixel(0.84);}) //0.0
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.3) //0.2
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{arm.setArmPos(0.51, 0.65);}) //0.3
//                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.65);})
//                .addTemporalMarker(()->{arm.setArmPos(0.52, 0.68);})
//                .lineToConstantHeading(YellowRight)
//                .addTemporalMarker(()->{arm.setArmPos(0.53,0.68);})
//                .waitSeconds(0.2)
//                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{ArmV2.DropPixel(1);}) //0.4
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3) //start
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeArmServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
//                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{arm.setArmPos(0.54, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(0.6,()->{arm.setArmPos(0.3, 0.175);})
//                .UNSTABLE_addTemporalMarkerOffset(0.7,()->{arm.setArmPos(0.15, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(0.54, 0.175);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
//                .waitSeconds(5)
                .resetConstraints()
                .build();

        TrajectorySequence ParkingOut = drive.trajectorySequenceBuilder(AutoTrajectoryRightYellow2.end())
                .lineToSplineHeading(new Pose2d(50, 12, -Math.PI/2))
                .lineToConstantHeading(new Vector2d(60, 12))
                .build();

        TrajectorySequence ParkingIn = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(51, 60))
                .turn(Math.PI/2)
                .build();

        TrajectorySequence AutoTrajectoryCenterPurple = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})
                .lineToSplineHeading(PurpleCenterPos)
                .build();

//                .lineToSplineHeading(StackCenterPos)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-34,12),0)
//                .splineToConstantHeading(new Vector2d(28,12),0)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(YellowCenter,0)
//                .strafeLeft(12)
//                .setReversed(false)
//                .resetConstraints()
//                //round1
//                .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
//                .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .lineToConstantHeading(StackCenter)
//                .resetConstraints()
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-34,12),0)
//                .splineToConstantHeading(new Vector2d(28,12),0)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(YellowCenter,0)
//
//                .resetConstraints()
//                .setReversed(false)
//                .waitSeconds(30)

//        TrajectorySequence AutoTrajectoryLeft = drive.trajectorySequenceBuilder(startPose)
//                .lineToSplineHeading(PurpleLeftPos)
//                .lineToSplineHeading(new Pose2d(-44 , 12, -Math.PI))
//                .lineToSplineHeading(StackLeftPos)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-34,12),0)
//                .splineToConstantHeading(new Vector2d(28,12),0)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(YellowLeft,0) //52
//                .strafeLeft(15)
//                .setReversed(false)
//                .resetConstraints()
//                //round1
//                .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
//                .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .lineToConstantHeading(StackLeft)
//                .resetConstraints()
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-34,12),0)
//                .splineToConstantHeading(new Vector2d(28,12),0)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(YellowLeft,0)
//
//                .resetConstraints()
//                .setReversed(false)
//                .waitSeconds(30)
//                .build();

        while (opModeInInit()) {
            slider.extendToHome();
            ArmV2.SetArmPosition(0.15, 0.16);
            Intake.SetArmPosition(0.5, 0.66);
            Intake.IntakePixel(0.8);
            ArmV2.DropPixel(0.8);
            Intake.CrankPosition(0.69);
            ArmV2.SliderLink(0.95);

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());
            if (currentRecognitions.size() != 0) {

                boolean objectFound = false;

                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    y = (recognition.getTop() + recognition.getBottom()) / 2;

                    objectFound = true;

                    telemetry.addLine("Beacon");
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    telemetry.update();

                    break;
                }

                if (objectFound) {

//                    Adjust values according to your bot and camera position
                    if (x >= 800 && x <= 1100) {
                        propPosition = "left";
                    } else if (x >= 500 && x <= 790) {
                        propPosition = "center";
                    } else if (x >= 200 && x <= 490) {
                        propPosition = "right";
                    }


                } else {
                    telemetry.addLine("Don't see the beacon :(");
                }
                telemetry.addData("position", propPosition);
                telemetry.update();
                telemetry.update();
            }
        }

        waitForStart();

        while (opModeIsActive()) {
            switch (currentState){
                case Start:
                    if (gamepad1.b|| propPosition == "right" ){ //
                        if (!drive.isBusy()) {
                            currentState = AutoTrajectoryRight.AutoTrajectoryRightPurple;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryRightPurple);
                        }
                    }
                    break;
                case AutoTrajectoryRightPurple:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPlacing;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing);
                    }
                    break;
                case CenterPathPlacing:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.AutoTrajectoryRightYellow;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryRightYellow);
                    }
                    break;
                case AutoTrajectoryRightYellow:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPicking;
                        drive.followTrajectorySequenceAsync(CenterPathPicking);
                    }
                    break;
                case CenterPathPicking:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPlacing2;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing2);
                    }
                    break;
                case CenterPathPlacing2:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.AutoTrajectoryRightYellow2;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryRightYellow2);
                    }
                    break;
                case AutoTrajectoryRightYellow2:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.ParkingOut;
                        drive.followTrajectorySequenceAsync(ParkingOut);
                    }
                    break;
                case ParkingOut:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            if (gamepad1.y){
//                drive.followTrajectorySequence(AutoTrajectoryCenter);
            }
            if (gamepad1.x){
//                drive.followTrajectorySequence(AutoTrajectoryLeft);
            }
            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            telemetry.addData("position",propPosition);
            drive.update();
            telemetry.update();
        }
        visionPortal.close();
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        telemetry.addLine("init done");
        telemetry.update();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.92f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }
    public void telem(){
        telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
        telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
        telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
        telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
        telemetry.update();
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
