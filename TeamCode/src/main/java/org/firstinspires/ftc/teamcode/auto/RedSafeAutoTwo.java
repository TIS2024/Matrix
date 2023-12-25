package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Monel_RedSafeAutoTwo")
public class RedSafeAutoTwo extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;

    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;

    public static double kp = 4.5, ki, kd = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(-39, -64, -Math.PI);
        drive.setPoseEstimate(startPose);

        while (opModeInInit()){
            slider.extendToHome();
            ArmV2.SetArmPosition(0.15, 0.19);
            Intake.SetArmPosition(0.4,0.66);
            Intake.IntakePixel(0.95);
            ArmV2.DropPixel(0.5);
            Intake.CrankPosition(0.69);
        }

        TrajectorySequence AutoTrajectoryRight = drive.trajectorySequenceBuilder(startPose)
                // right line
                .lineToSplineHeading(new Pose2d(-42,-32, -Math.PI))

                .addTemporalMarker(()->{arm.setArmPos(0.6, 0.19);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(0.6, 0.73);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.8);})
                .waitSeconds(1)

                //   towards pixel stack
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262);})

                .lineToSplineHeading(new Pose2d(-51 , -12, -Math.PI))

                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.CrankPosition(0.35);arm.setArmPos(0.3, 0.19);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.8)
                .addTemporalMarker(this::telem)
                .waitSeconds(1)
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.75);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.19);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{output_power = lifter_pid(kp, ki, kd, -10);if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.19);slider.extendTo(-10, output_power);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{output_power = lifter_pid(kp, ki, kd, 0);if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }})
                .addTemporalMarker(()->{Intake.IntakePixel(1);slider.extendTo(0, output_power);})
                .resetConstraints()
                .setReversed(true)

                //   towards backdrop
                .splineToConstantHeading(new Vector2d(-34,-12),0)
                .splineToConstantHeading(new Vector2d(28,-12),0)
                .splineToConstantHeading(new Vector2d(47,-40),0)
                .waitSeconds(1)

                .setReversed(false)
                .build();

        TrajectorySequence AutoTrajectoryCenter = drive.trajectorySequenceBuilder(startPose)
                // right line & towards pixel stack
                .lineToSplineHeading(new Pose2d(-51 , -12, -Math.PI))
                .waitSeconds(1)
                .setReversed(true)
                //   towards backdrop
                .splineToConstantHeading(new Vector2d(-34,-12),0)
                .splineToConstantHeading(new Vector2d(28,-12),0)
                .splineToConstantHeading(new Vector2d(47,-34),0)
                .waitSeconds(2)
                .setReversed(false)
                .build();

        TrajectorySequence AutoTrajectoryLeft = drive.trajectorySequenceBuilder(startPose)
                // right line
                .lineToSplineHeading(new Pose2d(-60,-32, -Math.PI))
                .waitSeconds(1)
                //   towards pixel stack
                .lineToSplineHeading(new Pose2d(-58 , -12, -Math.PI))
                .waitSeconds(1)
                .setReversed(true)
                //   towards backdrop
                .splineToConstantHeading(new Vector2d(-34,-12),0)
                .splineToConstantHeading(new Vector2d(28,-12),0)
                .splineToConstantHeading(new Vector2d(47,-28),0)
                .waitSeconds(2)
                .setReversed(false)
                .build();
        waitForStart();


        while (opModeIsActive()) {
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
            drive.update();
            telemetry.update();
        }
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
