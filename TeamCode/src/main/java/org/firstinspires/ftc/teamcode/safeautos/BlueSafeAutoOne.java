package org.firstinspires.ftc.teamcode.safeautos;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Config
@Autonomous(name = "BLUE_SafeAuto1", group = "Safe_Autos")
public class BlueSafeAutoOne extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
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
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(14, 62, -Math.PI);
        drive.setPoseEstimate(startPose);
        initTfod();

        while (opModeInInit()){
            slider.extendToHome();
            ArmV2.SetArmPosition(0.15, 0.16);
            Intake.SetArmPosition(0.5,0.66);
            Intake.IntakePixel(0.8);
            ArmV2.DropPixel(0.5);
            Intake.CrankPosition(0.69);
            ArmV2.SliderLink(0.95);
        }

        TrajectorySequence AutoTrajectoryRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(17.5 , 30))

                .addTemporalMarker(()->{Intake.CrankPosition(0.35);arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.CrankPosition(0.42);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);arm.setArmPos(0.5, 0.66);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .lineToConstantHeading(new Vector2d(52,27))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.2)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()

                //pixel intake // round 1
                .addTemporalMarker(() -> {arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.16);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(51, 60))
                .turn(Math.PI/2)
                .setReversed(false)
                .build();

        TrajectorySequence AutoTrajectoryCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(28 , 22))
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.51, 0.66);})
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.CrankPosition(0.5);})
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(50,36), 0)
                .waitSeconds(1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(1)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
//                .setReversed(false)

                //pixel intake // round 1
                .addTemporalMarker(() -> {arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.16);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(50, 60))
                .turn(Math.PI/2)
//                .strafeRight(10)
                .setReversed(false)
                .build();
        //pixel intake // round 2-------

        TrajectorySequence AutoTrajectoryLeft = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(35 , 28))
                .addTemporalMarker(()->{Intake.IntakePixel(1);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.66);})
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(51.5,43), 0)
                .waitSeconds(1)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(1)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
//                .setReversed(false)

                //pixel intake // round 1
                .addTemporalMarker(() -> {arm.setArmPos(0.4, 0.16);})
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {arm.setArmPos(0.15, 0.16);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(50.5, 60))
                .turn(Math.PI/2)
//                .strafeRight(6)
                .setReversed(false)
                //pixel intake // round 2------------------------------------------------------------

                .build();



        waitForStart();

        while (opModeIsActive()) {
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

                if(objectFound){

//                    Adjust values according to your bot and camera position
//                    if(x>=800 && x<=1100){
//                        propPosition  = "left";
//                        drive.followTrajectorySequence(AutoTrajectoryLeft);
//                    }
//                    else if(x>=500 && x<=790){
//                        propPosition = "center";
//                        drive.followTrajectorySequence(AutoTrajectoryCenter);
//                    }
//                    else if(x>=200 && x<=490) {
//                        propPosition = "right";
//                        drive.followTrajectorySequence(AutoTrajectoryRight);
//                    }


                }
                else{
                    telemetry.addLine("Don't see the beacon :(");
                }
            }
            else{
                telemetry.addLine("Don't see the beacon :(");
            }
            if (gamepad1.b){
                drive.followTrajectorySequence(AutoTrajectoryRight);
            }
            if (gamepad1.y){
                drive.followTrajectorySequence(AutoTrajectoryCenter);
            }
            if (gamepad1.x){
                drive.followTrajectorySequence(AutoTrajectoryLeft);
            }
            telemetry.addData("LeftFrontCurrent", drive.getMotorCurrent().get(0));
            telemetry.addData("RightFrontCurrent", drive.getMotorCurrent().get(1));
            telemetry.addData("LeftRearCurrent", drive.getMotorCurrent().get(2));
            telemetry.addData("RightRearCurrent", drive.getMotorCurrent().get(3));
            visionPortal.close();
            telemetry.addData("position",propPosition);
            sleep(500);
            drive.update();
            telemetry.update();
        }
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

        // C reate the vision portal by using a builder.


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
        //tfod.setMinResultConfidence(0.75f);

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
