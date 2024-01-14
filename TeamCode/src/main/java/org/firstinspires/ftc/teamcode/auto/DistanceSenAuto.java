package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class DistanceSenAuto extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    ArmV2 arm = null;
    Hanger hanger = null;
    Intake intake = null;
    private DistanceSensor sensorDistance, sensorDistance2, sensorDistance3;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
        sensorDistance3 = hardwareMap.get(DistanceSensor.class, "sensor_distance3");

        Pose2d startPose=new Pose2d(-39, -64, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence first = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-51,-24, 0))
                .lineToSplineHeading(new Pose2d(-51 , -12, -Math.PI))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34,-12),0)
                .splineToConstantHeading(new Vector2d(36,-12),0)
                .addTemporalMarker(()->{
                    if(sensorDistance.getDistance(DistanceUnit.INCH) < 10) {drive.setMotorPowers(0, 0,0,0);}})
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                .splineToConstantHeading(new Vector2d(53,-38),0)
                .setReversed(false)
                .resetConstraints()
                .build();

        while (opModeInInit()){
////            slider.extendToHome();
//            ArmV2.SetArmPosition(0.15, 0.16);
//            Intake.SetArmPosition(0.5, 0.66);
//            Intake.IntakePixel(0.8);
//            ArmV2.DropPixel(0.8);
//            Intake.CrankPosition(0.69);
//            ArmV2.SliderLink(0.95);
        }

        waitForStart();
        drive.followTrajectorySequence(first);
        drive.update();
        while(opModeIsActive()){
            telemetry.addData("Xpose", drive.getPoseEstimate());
            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
            telemetry.addData("range1", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range1", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

            telemetry.addData("range2", String.format("%.01f mm", sensorDistance2.getDistance(DistanceUnit.MM)));
            telemetry.addData("range2", String.format("%.01f in", sensorDistance2.getDistance(DistanceUnit.INCH)));

            telemetry.addData("range3", String.format("%.01f mm", sensorDistance3.getDistance(DistanceUnit.MM)));
            telemetry.addData("range3", String.format("%.01f in", sensorDistance3.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }
}
