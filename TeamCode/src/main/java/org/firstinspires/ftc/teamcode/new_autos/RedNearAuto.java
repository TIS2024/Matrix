package org.firstinspires.ftc.teamcode.new_autos;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

@Autonomous(name = "RedNearAuto_1Cycle", group = "Cycle_Autos")
@Disabled
public class RedNearAuto extends LinearOpMode {
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

    public static Pose2d PurpleLeftPos, YellowLeftPos, StackLeftPos;
    public static Vector2d PurpleLeft, YellowLeft, StackLeft;


    public static double wristPlay1 = -0.01, wristPlay2 = -0.01;

    enum AutoTrajectoryLeft {
        Start,
        AutoTrajectoryLeft_Purple_Yellow,
        CenterPathPicking,
        CenterPathPlacing,
        WhiteDrop_Left,
        CenterPathPicking2,
        CenterPathPlacing2,
        WhiteDrop2_Left,
        ParkingIn,
        IDLE
    }

    AutoTrajectoryLeft currentLeftState = AutoTrajectoryLeft.Start;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);
        initTfod();


        //TODO Left Trajectories
        TrajectorySequence AutoTrajectoryLeft_Purple_Yellow = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                .lineToConstantHeading(new Vector2d(16 , -29))

                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{arm.setArmPos(0.4, 0.175);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.45);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);arm.setArmPos(0.54, 0.66);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(50,-25), 0)

                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.3, 0.175);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.175);})
                .setReversed(false)
                .resetConstraints()
                .build();

        TrajectorySequence CenterPathPicking = drive.trajectorySequenceBuilder(AutoTrajectoryLeft_Purple_Yellow.end())
                .splineToConstantHeading(new Vector2d(18,-7), -Math.PI)
                .splineToConstantHeading(new Vector2d(-33,-10), -Math.PI)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.16);})
                .addTemporalMarker(()->{Intake.CrankPosition(0.38);})
                .build();


        TrajectorySequence CenterPathPlacing = drive.trajectorySequenceBuilder(CenterPathPicking.end())
                .build();
        TrajectorySequence WhiteDrop_Left = drive.trajectorySequenceBuilder(CenterPathPlacing.end())
                .build();
        TrajectorySequence CenterPathPicking2 = drive.trajectorySequenceBuilder(WhiteDrop_Left.end())
                .build();
        TrajectorySequence CenterPathPlacing2 = drive.trajectorySequenceBuilder(CenterPathPicking2.end())
                .build();
        TrajectorySequence WhiteDrop2_Left = drive.trajectorySequenceBuilder(CenterPathPlacing2.end())
                .build();
        TrajectorySequence ParkingOut = drive.trajectorySequenceBuilder(WhiteDrop2_Left.end())
                .build();
        TrajectorySequence ParkingIn = drive.trajectorySequenceBuilder(WhiteDrop2_Left.end())
                .lineToConstantHeading(new Vector2d(48, -60))
                .turn(-Math.PI/2)
                .build();

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
                        propPosition = "right";
                    } else if (x >= 500 && x <= 790) {
                        propPosition = "center";
                    } else if (x >= 200 && x <= 490) {
                        propPosition = "left";
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

            //LEFT TRAJECTORY
            switch (currentLeftState){
                case Start:
                    if (gamepad1.x || propPosition == "left"){
                        if(!drive.isBusy()){
                            currentLeftState = AutoTrajectoryLeft.AutoTrajectoryLeft_Purple_Yellow;
                            drive.followTrajectorySequenceAsync(AutoTrajectoryLeft_Purple_Yellow);
                        }
                    }
                    break;
                case AutoTrajectoryLeft_Purple_Yellow:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPicking;
                        drive.followTrajectorySequenceAsync(CenterPathPicking);
                    }
                    break;
                case CenterPathPicking:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPlacing;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing);
                    }
                    break;
                case CenterPathPlacing:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.WhiteDrop_Left;
                        drive.followTrajectorySequenceAsync(WhiteDrop_Left);
                    }
                    break;
                case WhiteDrop_Left:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPicking2;
                        drive.followTrajectorySequenceAsync(CenterPathPicking2);
                    }
                    break;
                case CenterPathPicking2:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.CenterPathPlacing2;
                        drive.followTrajectorySequenceAsync(CenterPathPlacing2);
                    }
                    break;
                case CenterPathPlacing2:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.WhiteDrop2_Left;
                        drive.followTrajectorySequenceAsync(WhiteDrop2_Left);
                    }
                    break;
                case WhiteDrop2_Left:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.ParkingIn;
                        drive.followTrajectorySequenceAsync(ParkingIn);
                    }
                    break;
                case ParkingIn:
                    if(!drive.isBusy()){
                        currentLeftState = AutoTrajectoryLeft.IDLE;
                    }
                    break;
                case IDLE:
                    break;
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