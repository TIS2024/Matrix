package org.firstinspires.ftc.teamcode.new_autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RedAuto_2cycles", group = "Cycle_Autos")
@Config
public class RedAuto_2cycles extends LinearOpMode {
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

        slider.extendToHome();
        ArmV2.SetArmPosition(0.15,0.155);
        Intake.SetArmPosition(0.5,0.66);
        Intake.IntakePixel(0.75);
        ArmV2.DropPixel(0.75);
        Intake.CrankPosition(0.69);

        Pose2d  startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(this::telem)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.4);Intake.intakeWristServo.setPosition(0.5);})
                //backdrop
                .lineToConstantHeading(new Vector2d(30 , -36))

                .UNSTABLE_addTemporalMarkerOffset(-0.60,()->{Intake.IntakePixel(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.30,()->{arm.setArmPos(0.5, 0.155);})
                .UNSTABLE_addTemporalMarkerOffset(-0.10,()->{arm.setArmPos(0.5, 0.66);})

                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30), 0)

                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.2)//0.55
                .addTemporalMarker(this::telem)
                .resetConstraints()
                .setReversed(false)

                //pixel intake // round 1
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{ArmV2.wristServo.setPosition(0.155);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{arm.setArmPos(0.15, 0.155);})

                .splineToConstantHeading(new Vector2d(18,-8), -Math.PI)
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(-34,-8), -Math.PI)
                .addTemporalMarker(this::telem)

                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()->{Intake.intakeArmServo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.265);})

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{arm.setArmPos(0.25, 0.155);})
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{Intake.CrankPosition(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.8);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.intakeArmServo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.269);})
                .waitSeconds(0.2)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.CrankPosition(0.69);})
                .resetConstraints()
                .setReversed(true)

                //backdrop and intake pixel
                .splineToConstantHeading(new Vector2d(-34,-10),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(0.95,()->{ArmV2.wristServo.setPosition(0.155);})
                .UNSTABLE_addTemporalMarkerOffset(1.05, ()->{arm.setArmPos(0.15, 0.155);})

                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-10),0)
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)

                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{ArmV2.DropPixel(0.5);arm.setArmPos(0, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.155);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.66);})
                .waitSeconds(0.3) //0.6
                .addTemporalMarker(()->{slider.extendTo(230, 0.8);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
                .waitSeconds(0.4) //0.8
//                .lineToConstantHeading(new Vector2d(47.3, -35))
//                .addTemporalMarker(()->{ArmV2.wristServo.setPosition(0.05);ArmV2.ArmV2Servo.setPosition(0.545);})
//                .waitSeconds(0.2) //0.4
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.8);})
                .waitSeconds(0.05)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeWristServo.setPosition(0.38);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.80,()->{Intake.intakeWristServo.setPosition(0.395);Intake.intakeArmServo.setPosition(0.513);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{arm.setArmPos(0.3, 0.155);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{arm.setArmPos(0.15, 0.155);})
                .setReversed(false)

                //pixel intake // round 2------------------------------------------------------------
                .splineToConstantHeading(new Vector2d(18,-8),-Math.PI)
                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(-34, -8), -Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Intake.IntakePixel(1);})
                .addTemporalMarker(this::telem)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->{arm.setArmPos(0.25, 0.155);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.1)
                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
                .waitSeconds(0.2)
                .setReversed(true)

                // intake pixel into bot
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.CrankPosition(0.69);})
                .resetConstraints()
                .setReversed(true)
                //backdrop and intake pixel

                .splineToConstantHeading(new Vector2d(-34,-10),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.66);Intake.intakeArmServo.setPosition(0.4);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmServo.setPosition(0.75);})
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmServo.setPosition(1);Intake.intakeWristServo.setPosition(0.45);})
                .UNSTABLE_addTemporalMarkerOffset(1.00,()->{ArmV2.wristServo.setPosition(0.155);})
                .UNSTABLE_addTemporalMarkerOffset(1.10, ()->{arm.setArmPos(0.15, 0.155);})

                .addTemporalMarker(this::telem)
                .splineToConstantHeading(new Vector2d(18,-10),0)
                .addTemporalMarker(this::telem)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(47.5,-30),0)

                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{ArmV2.DropPixel(0.5);arm.setArmPos(0, 0.155);slider.extendTo(-10, 1);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);arm.setArmPos(0.15, 0.155);slider.extendTo(0, 1);})
                .addTemporalMarker(this::telem)
                .waitSeconds(0.2)

                //place pixel on backdrop
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.155);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{arm.setArmPos(0.5, 0.66);})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{slider.extendTo(200, 0.9);})
                .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
                .waitSeconds(0.1)
                .addTemporalMarker(()->{slider.extendTo(0, 0.9);})
                .addTemporalMarker(()->{arm.setArmPos(0.15, 0.155);})
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeWristServo.setPosition(0.38);}) //0.0
                .UNSTABLE_addTemporalMarkerOffset(0.65,()->{Intake.intakeArmServo.setPosition(0.5);Intake.intakeWristServo.setPosition(0.66);})
                .UNSTABLE_addTemporalMarkerOffset(0.80,()->{Intake.intakeWristServo.setPosition(0.265);Intake.intakeArmServo.setPosition(0.645);})
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{arm.setArmPos(0.3, 0.155);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{arm.setArmPos(0.15, 0.155);})
                .setReversed(false)

                //pixel intake // round 3-----------------------------------------------------------
//                .splineToConstantHeading(new Vector2d(18,-8),-Math.PI)
////                .addTemporalMarker(()->{Intake.intakeWristServo.setPosition(0.4);})
//                .addTemporalMarker(this::telem)
////                .UNSTABLE_addTemporalMarkerOffset(-0.7,()->{Intake.intakeArmV2Servo.setPosition(0.7);Intake.intakeWristServo.setPosition(0.65);}) //0.0
////                .UNSTABLE_addTemporalMarkerOffset(-0.35,()->{Intake.intakeArmV2Servo.setPosition(0.4);}) //0.35
////                .UNSTABLE_addTemporalMarkerOffset(-0.20,()->{Intake.intakeWristServo.setPosition(0.252);Intake.intakeArmV2Servo.setPosition(0.636);})//0.633-0.2515//ArmV2->0.64 //0.50
////                .UNSTABLE_addTemporalMarkerOffset(-0.20,()->{ArmV2.ArmV2Servo.setPosition(0.15);ArmV2.wristServo.setPosition(0.73);}) //0.2
//
//                .splineToConstantHeading(new Vector2d(-31,-8),-Math.PI)
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{Intake.IntakePixel(1);})
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{Intake.CrankPosition(0.5);})
//                .addTemporalMarker(this::telem)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(-51.7,-24.2),-Math.PI)
//                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{Intake.CrankPosition(0.38);})
//                .addTemporalMarker(this::telem)
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{ArmV2.ArmV2Servo.setPosition(0.30);ArmV2.wristServo.setPosition(0.73);})
//                .waitSeconds(0.1)
//                .addTemporalMarker(()->{Intake.IntakePixel(0.75);})
//                .waitSeconds(0.2) //0.3
//                .addTemporalMarker(()->{Intake.intakeArmV2Servo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.2695);})
//                .waitSeconds(0.05)
//                .setReversed(true)
//
//                // intake pixel into bot
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{Intake.intakeArmV2Servo.setPosition(0.645);Intake.intakeWristServo.setPosition(0.2595);Intake.CrankPosition(0.7);})
//                .resetConstraints()
//                .setReversed(true)
//                //backdrop and intake pixel
//                .splineToConstantHeading(new Vector2d(-31,-8),0)
//                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Intake.intakeWristServo.setPosition(0.65);Intake.intakeArmV2Servo.setPosition(0.55);})
//                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{Intake.intakeArmV2Servo.setPosition(0.8);})
//                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{Intake.intakeArmV2Servo.setPosition(1);Intake.intakeWristServo.setPosition(0.44);})
//                .UNSTABLE_addTemporalMarkerOffset(1.00,()->{ArmV2.wristServo.setPosition(0.73);})
//                .UNSTABLE_addTemporalMarkerOffset(1.10, ()->{ArmV2.ArmV2Servo.setPosition(0.15);})
//                .addTemporalMarker(this::telem)
//                .splineToConstantHeading(new Vector2d(18,-8),0)
//                .addTemporalMarker(this::telem)
//                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(35))
//                .splineToConstantHeading(new Vector2d(47.5,-30),0)
//                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{ArmV2.DropPixel(0.45);ArmV2.ArmV2Servo.setPosition(0);slider.extendTo(-20, 1);})
//                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{Intake.IntakePixel(1);slider.extendTo(0, 1);})
//                .addTemporalMarker(this::telem)
//                .waitSeconds(0.2)
//
//                //place pixel on backdrop
//                .addTemporalMarker(()->{ArmV2.wristServo.setPosition(0.05);ArmV2.ArmV2Servo.setPosition(0.55);})
//                .waitSeconds(0.3)//0.6
//                .addTemporalMarker(()->{slider.extendTo(250, 1);})
//                .addTemporalMarker(()->{ArmV2.DropPixel(0.75);})
//                .waitSeconds(0.4)//0.8
////                .lineToConstantHeading(new Vector2d(47.3, -30))
//                .addTemporalMarker(()->{ArmV2.wristServo.setPosition(0.05);ArmV2.ArmV2Servo.setPosition(0.545);})
//                .waitSeconds(0.2)//0.4
//                .addTemporalMarker(()->{ArmV2.DropPixel(1);})
//                .waitSeconds(0.1)
//                .addTemporalMarker(()->{slider.extendTo(0, 1);})
//                .waitSeconds(0.05)
//                .addTemporalMarker(()->{ArmV2.ArmV2Servo.setPosition(0.15);ArmV2.wristServo.setPosition(0.73);})
//                .resetConstraints()
//                .setReversed(false)
//                .waitSeconds(200)
                .build();


        waitForStart();

        drive.followTrajectorySequence(first);
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
