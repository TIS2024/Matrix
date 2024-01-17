package org.firstinspires.ftc.teamcode.auto.safeautos;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.DistanceSenAuto;
import org.firstinspires.ftc.teamcode.auto.new_autos.RedFarAuto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
@Autonomous(name = "REDFarAuto_Safe", group = "1Safe_Autos")
public class RedSafeFarAuto extends LinearOpMode {
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

    public static double stackDiff = 0;
    public static double yellowDiff = 3;
    public static Pose2d PurpleRightPos = new Pose2d(-38,-32, -Math.PI), YellowRightPos, StackRightPos = new Pose2d(-51 , -12 + stackDiff, -Math.PI); //53
    public static Vector2d PurpleRight, YellowRight = new Vector2d(53.5 - yellowDiff,-41.5), StackRight = new Vector2d(-51, -12.5);

    public static Pose2d PurpleLeftPos = new Pose2d(-40,-30, 0), YellowLeftPos, StackLeftPos = new Pose2d(-51 , -11 + stackDiff, -Math.PI);//54, //-44
    public static Vector2d PurpleLeft, YellowLeft = new Vector2d(53.5 - yellowDiff,-28.5), StackLeft = new Vector2d(-51, -12); //48

    public static Pose2d PurpleCenterPos = new Pose2d(-53,-24, 0), YellowCenterPos, StackCenterPos = new Pose2d(-51 , -12, -Math.PI); //51
    public static Vector2d PurpleCenter, YellowCenter = new Vector2d(53.5 - yellowDiff,-33), StackCenter = new Vector2d(-51, -12); //38


    public static double wristPlay1 = -0.01, wristPlay2 = 0.00;
    double errorPose = 0;


    public enum AutoTrajectoryRight {
        Start,
        AutoTrajectoryRightPurple,
        CenterPathPlacing,
        poseCorrection,
        AutoTrajectoryRightYellow,
        ParkingOut,
        IDLE
    }
    enum AutoTrajectoryCenter {
        Start,
        AutoTrajectoryCenterPurple,
        CenterPathPlacing_Center,
        AutoTrajectoryCenterYellow,
        ParkingOut,
        IDLE
    }
    enum AutoTrajectoryLeft {
        Start,
        AutoTrajectoryLeftPurple,
        CenterPathPlacing_Left,
        AutoTrajectoryLeftYellow,
        ParkingOut,
        IDLE
    }
    AutoTrajectoryRight currentState = AutoTrajectoryRight.Start;
    AutoTrajectoryCenter currentState2 = AutoTrajectoryCenter.Start;
    AutoTrajectoryLeft currentState3 = AutoTrajectoryLeft.Start;

    private DistanceSensor sensorDistance, sensorDistance2, sensorDistance3;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(-39, -64, 0);
        drive.setPoseEstimate(startPose);
        initTfod();

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
        sensorDistance3 = hardwareMap.get(DistanceSensor.class, "sensor_distance3");



        // TODO Left Trajectories
        TrajectorySequence AutoTrajectoryLeftPurple = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})

                .lineToSplineHeading(PurpleRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
                .build();

        TrajectorySequence CenterPathPlacing_Left = drive.trajectorySequenceBuilder(AutoTrajectoryLeftPurple.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + wristPlay1);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.4);})

                .lineToSplineHeading(StackRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(0.15, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.175);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

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

                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                .build();

        TrajectorySequence AutoTrajectoryLeftYellow = drive.trajectorySequenceBuilder(CenterPathPlacing_Left.end())
                .lineToConstantHeading(new Vector2d(53.5, -40))

                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.54, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.54, 0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.48, 0.68);}) //50
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{arm.setArmPos(0.50,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{arm.setArmPos(0.51,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{arm.setArmPos(0.52,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{arm.setArmPos(0.53,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{arm.setArmPos(0.54,0.68);})

                .lineToConstantHeading(YellowLeft)

                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.4, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
                .resetConstraints()
                .build();

        TrajectorySequence ParkingOut = drive.trajectorySequenceBuilder(AutoTrajectoryLeftYellow.end())
//                .lineToSplineHeading(new Pose2d(50, -12, -Math.PI/2))
                .lineToConstantHeading(new Vector2d(50, -12))
                .build();

        TrajectorySequence ParkingIn = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(51, -60))
                .turn(Math.PI/2)
                .build();








        //TODO Center Trajectories
        TrajectorySequence AutoTrajectoryCenterPurple = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})

                .lineToSplineHeading(PurpleCenterPos)

                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.4);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
                .lineToConstantHeading(new Vector2d(-55, -18))
                .build();

        TrajectorySequence CenterPathPlacing_Center = drive.trajectorySequenceBuilder(AutoTrajectoryCenterPurple.end())
                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + wristPlay1);})
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.41);})

                .lineToSplineHeading(StackRightPos)

                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-34,-12),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(0.15, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.175);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

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

                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                .build();

        TrajectorySequence AutoTrajectoryCenterYellow = drive.trajectorySequenceBuilder(CenterPathPlacing_Center.end())
                .lineToConstantHeading(new Vector2d(50.5, -40))

                .UNSTABLE_addTemporalMarkerOffset(-1,()->{arm.setArmPos(0.54, 0.175);})
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{arm.setArmPos(0.54, 0.68);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);}) //50
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{arm.setArmPos(0.50,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{arm.setArmPos(0.51,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{arm.setArmPos(0.52,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{arm.setArmPos(0.53,0.68);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{arm.setArmPos(0.54,0.68);})


                .lineToConstantHeading(YellowCenter)

                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.4, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
                .resetConstraints()
                .build();





        //TODO Right Trajectories




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
                    if( x>=800 && x<=1100){
                        propPosition  = "left";
                    }
                    else if(x>=500 && x<=790){
                        propPosition = "center";
                    }
                    else if(x>=200 && x<=490) {
                        propPosition = "right";
                    }


                } else {
                    telemetry.addLine("Don't see the beacon :(");
                }
                telemetry.addData("position", propPosition);
                telemetry.update();
            }
            sleep(20);
        }
        visionPortal.close();

        waitForStart();

        while (opModeIsActive()) {

            double reqDist = 2;
            double backDropDistance = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH) + sensorDistance3.getDistance(DistanceUnit.INCH))/3;
            double error = backDropDistance - reqDist;
            double factor = 1.2; //1.17

            double x = drive.getPoseEstimate().getX() + (error * factor);
            double y = drive.getPoseEstimate().getY();
            double heading = drive.getPoseEstimate().getHeading();

            //RIGHT TRAJECTORY
            switch (currentState){
                case Start:
                    if (gamepad1.b || propPosition == "right"){ //
                        if (!drive.isBusy()) {
                            currentState = AutoTrajectoryRight.AutoTrajectoryRightPurple;
                            TrajectorySequence AutoTrajectoryRightPurple = drive.trajectorySequenceBuilder(startPose)
                                    .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})

                                    .lineToSplineHeading(PurpleLeftPos)

                                    .addTemporalMarker(()->{arm.setArmPos(0.3, 0.175);})
                                    .waitSeconds(0.2)
                                    .addTemporalMarker(()->{Intake.CrankPosition(0.5);})
                                    .waitSeconds(0.2)
                                    .addTemporalMarker(()->{Intake.IntakePixel(1);})
                                    .waitSeconds(0.2)
                                    .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                                    .addTemporalMarker(()->{intake.setArm(0.5, 0.66);})
                                    .build();
                            drive.followTrajectorySequenceAsync(AutoTrajectoryRightPurple);
                        }
                    }
                    break;
                case AutoTrajectoryRightPurple:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.CenterPathPlacing;
                        TrajectorySequence CenterPathPlacing = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))

                                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.67);Intake.intakeWristServo.setPosition(0.24 + wristPlay1);})
                                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                                .addTemporalMarker(()->{Intake.CrankPosition(0.4);})

                                .lineToSplineHeading(StackRightPos)

                                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                                .waitSeconds(0.3)
                                .addTemporalMarker(()->{intake.setArm(0.69, 0.4);})
                                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(-34,-12),0)

                                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{Intake.intakeArmServo.setPosition(0.75);})
                                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                                .UNSTABLE_addTemporalMarkerOffset(1, ()->{arm.setArmPos(0.15, 0.175);})
                                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{Intake.IntakePixel(1);ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.175);slider.extendTo(-10, 1);})
                                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{slider.extendTo(0, 1);})

                                .splineToConstantHeading(new Vector2d(36,-12),0) //28
                                .lineToConstantHeading(new Vector2d(36, -32))
                                .build();

                        drive.followTrajectorySequenceAsync(CenterPathPlacing);
                    }
                    break;
                case CenterPathPlacing:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.poseCorrection;
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(error * factor)
                                .build());
                        errorPose = error*factor;

                    }
                    break;
                case poseCorrection:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.AutoTrajectoryRightYellow;
                        TrajectorySequence AutoTrajectoryRightYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                .back(1.5)

                                .addTemporalMarker(()->{arm.setArmPos(0.54, 0.175);})
                                .waitSeconds(0.5)
                                .addTemporalMarker(()->{arm.setArmPos(0.54, 0.68);})
                                .addTemporalMarker(()->{ArmV2.DropPixel(0.84);})
                                .waitSeconds(0.2)
                                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);}) //0.51
                                .waitSeconds(0.1)
                                .strafeLeft(10)

                                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{arm.setArmPos(0.50,0.68);})
                                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{arm.setArmPos(0.51,0.68);})
                                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{arm.setArmPos(0.52,0.68);})
                                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{arm.setArmPos(0.53,0.68);})
                                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{arm.setArmPos(0.54,0.68);})
                                .waitSeconds(0.2)
                                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.38);})
                                .UNSTABLE_addTemporalMarkerOffset(0.4,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                                .addTemporalMarker(()->{arm.setArmPos(0.49, 0.68);})
                                .waitSeconds(0.2)
                                .addTemporalMarker(()->{arm.setArmPos(0.4, 0.175);})
                                .waitSeconds(0.2)
                                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.175);})
                                .resetConstraints()
                                .build();
                        drive.followTrajectorySequenceAsync(AutoTrajectoryRightYellow);
                    }
                    break;
                case AutoTrajectoryRightYellow:
                    if (!drive.isBusy()) {
                        currentState = AutoTrajectoryRight.ParkingOut;
                        TrajectorySequence ParkingOut2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(new Vector2d(50, -12))
                                .build();
                        drive.followTrajectorySequenceAsync(ParkingOut2);
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


            //CENTER TRAJECTORY
            switch (currentState2){
                case Start:
                    if (gamepad1.y || propPosition == "center"){ //
                        if (!drive.isBusy()) {
                            currentState2 = AutoTrajectoryCenter.AutoTrajectoryCenterPurple;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryCenterPurple);
                        }
                    }
                    break;
                case AutoTrajectoryCenterPurple:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.CenterPathPlacing_Center;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Center);
                    }
                    break;
                case CenterPathPlacing_Center:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.AutoTrajectoryCenterYellow;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryCenterYellow);
                    }
                    break;
                case AutoTrajectoryCenterYellow:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.ParkingOut;
                        drive.followTrajectorySequenceAsync(ParkingOut);
                    }
                    break;
                case ParkingOut:
                    if (!drive.isBusy()) {
                        currentState2 = AutoTrajectoryCenter.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }


            //LEFT TRAJECTORY
            switch (currentState3){
                case Start:
                    if (gamepad1.x || propPosition == "left"){ //
                        if (!drive.isBusy()) {
                            currentState3 = AutoTrajectoryLeft.AutoTrajectoryLeftPurple;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryLeftPurple);
                        }
                    }
                    break;
                case AutoTrajectoryLeftPurple:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.CenterPathPlacing_Left;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing_Left);
                    }
                    break;
                case CenterPathPlacing_Left:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.AutoTrajectoryLeftYellow;
                        drive.followTrajectorySequenceAsync(AutoTrajectoryLeftYellow);
                    }
                    break;
                case AutoTrajectoryLeftYellow:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.ParkingOut;
                        drive.followTrajectorySequenceAsync(ParkingOut);
                    }
                    break;
                case ParkingOut:
                    if (!drive.isBusy()) {
                        currentState3 = AutoTrajectoryLeft.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            telemetry.addData("X-Pos", drive.getPoseEstimate().getX());
            telemetry.addData("YPos", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
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
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));


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
