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
@Autonomous (name = "MonelRedAutoOne")
public class MonelRedAutoOne extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new ArmV2(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);

        while (opModeInInit()){
            slider.extendToHome();
            ArmV2.SetArmPosition(0.15, 0.19);
            Intake.SetArmPosition(0.4,0.66);
            Intake.IntakePixel(0.80);
            ArmV2.DropPixel(0.8);
            Intake.CrankPosition(0.69);
        }

        TrajectorySequence AutoTrajectory = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})

                //backdrop
                .lineToConstantHeading(new Vector2d(30 , -36))
                .UNSTABLE_addTemporalMarkerOffset(-0.60,()->{Intake.IntakePixel(0.95);})
                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{arm.setArmPos(0.55, 0.22);})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{arm.setArmPos(0.55, 0.73);})
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30), 0)
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{ArmV2.DropPixel(0.95);})
                .waitSeconds(0.2)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 1
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{arm.setArmPos(0.2, 0.22);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{arm.setArmPos(0.15, 0.22);})
//                .splineToConstantHeading(new Vector2d(18,-8), -Math.PI)
                .lineToConstantHeading(new Vector2d(18,-8))
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(-34,-8), -Math.PI)
                .addTemporalMarker(this::telem)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262);}) //0.633-0.2515 //arm->0.64
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{arm.setArmPos(0.25, 0.22);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.269);}) //0.645-0.2595
                .waitSeconds(0.2)
                .addTemporalMarker(this::telem)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmServo.setPosition(0.645);Intake.CrankPosition(0.69);})
                .resetConstraints()
                .setReversed(true)
                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-10),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.72);})
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(0.95,()->{arm.setArmPos(0.15, 0.22);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-10),0)
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.22);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.22);if(ArmV2.armServoTwo.getPosition()==0.55){arm.setArmPos(0.5, 0.73);}})
                .waitSeconds(0.3) //0.6
                .addTemporalMarker(()->{slider.extendTo(230, 0.8);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
                .waitSeconds(0.4) //0.8
//                .lineToConstantHeading(new Vector2d(47.3, -35))
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.73);})
                .waitSeconds(0.2) //0.4
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.8);})
                .waitSeconds(0.05)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.95);Intake.intakeWristServo.setPosition(0.4);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})//0.375-0.513//arm->0.52 //0.50
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{arm.setArmPos(0.15, 0.22);})//0.2
                .setReversed(false)
                //pixel intake // round 2------------------------------------------------------------
                .build();

        TrajectorySequence AutoTrajectory2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(0.95);})

                //backdrop
//                .lineToConstantHeading(new Vector2d(30 , -36))

//                .UNSTABLE_addTemporalMarkerOffset(-0.60,()->{Intake.IntakePixel(0.95);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{arm.setArmPos(0.55, 0.22);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{arm.setArmPos(0.55, 0.73);})
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{Intake.IntakePixel(0.95);})
//                .addTemporalMarker(this::telem)

//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(47.5,-30), 0)

                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.19);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.73);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.95);})
                .waitSeconds(0.5)

//                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{ArmV2.DropPixel(0.95);})

                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.19);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.19);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.19);})
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 1
//                .lineToConstantHeading(new Vector2d(18,-8))
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(-34,-8), -Math.PI)
//                .addTemporalMarker(this::telem)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)

                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.CrankPosition(0.35);arm.setArmPos(0.25, 0.19);})
                .waitSeconds(1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(1)
                .addTemporalMarker(this::telem)

                // intake pixel into bot
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.75);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .waitSeconds(1)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.22);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.19);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .resetConstraints()

//                .splineToConstantHeading(new Vector2d(-34,-10),0)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(18,-10),0)
//                .addTemporalMarker(this::telem)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(47.5,-30),0)

                //place pixel on backdrop
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.19);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.73);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slider.extendTo(230, 0.8);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
                .waitSeconds(0.4) //0.8
                .addTemporalMarker(()->{arm.setArmPos(0.48, 0.73);})
                .waitSeconds(0.2) //0.4
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.8);})
                .waitSeconds(2)
                .resetConstraints()
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.95);Intake.intakeWristServo.setPosition(0.38);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .waitSeconds(2)
                .addTemporalMarker(()->{ArmV2.wristServo.setPosition(0.22);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.19);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.19);})
                .setReversed(false)
                        .build();

        TrajectorySequence AutoTrajectory3 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.55);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(0.95);})

                //backdrop
//                .lineToConstantHeading(new Vector2d(30 , -36))

//                .UNSTABLE_addTemporalMarkerOffset(-0.60,()->{Intake.IntakePixel(0.95);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{arm.setArmPos(0.55, 0.22);})
//                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{arm.setArmPos(0.55, 0.73);})
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{Intake.IntakePixel(0.95);})
//                .addTemporalMarker(this::telem)

//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(47.5,-30), 0)

                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.19);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.73);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.95);})
                .waitSeconds(0.3)

//                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{ArmV2.DropPixel(0.95);})

                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.19);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.19);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.19);})
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 1
//                .lineToConstantHeading(new Vector2d(18,-8))
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(-34,-8), -Math.PI)
//                .addTemporalMarker(this::telem)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)

                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.636);Intake.intakeWristServo.setPosition(0.262);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.CrankPosition(0.35);arm.setArmPos(0.25, 0.19);})
                .waitSeconds(0.8)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.8)
                .addTemporalMarker(this::telem)

                // intake pixel into bot
                .addTemporalMarker(()->{Intake.CrankPosition(0.69);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.75);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.22);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ArmV2.DropPixel(0.5);arm.setArmPos(0.1, 0.19);slider.extendTo(-10, 1);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .resetConstraints()

//                .splineToConstantHeading(new Vector2d(-34,-10),0)
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(18,-10),0)
//                .addTemporalMarker(this::telem)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(47.5,-30),0)

                //place pixel on backdrop
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.19);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.55, 0.73);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{slider.extendTo(230, 0.8);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
                .waitSeconds(0.2) //0.8
                .addTemporalMarker(()->{arm.setArmPos(0.48, 0.73);})
                .waitSeconds(0.2) //0.4
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.8);})
                .waitSeconds(0.5)
                .resetConstraints()
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.95);Intake.intakeWristServo.setPosition(0.38);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ArmV2.wristServo.setPosition(0.22);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{arm.setArmPos(0.3, 0.19);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.19);})
                .setReversed(false)
                .build();

        waitForStart();

//        drive.followTrajectorySequence(AutoTrajectory);
//        drive.followTrajectorySequence(AutoTrajectory2);
        drive.followTrajectorySequence(AutoTrajectory3);
        while (opModeIsActive()) {
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
}
