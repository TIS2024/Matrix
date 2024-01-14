package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity RedAuto = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, -64, 0))
                                //right 50pts
//                                        .lineToSplineHeading(new Pose2d(-35,32, -Math.PI))
//                                        .lineToSplineHeading(new Pose2d(-32 , 12, -Math.PI))
//                                        .lineToSplineHeading(new Pose2d(-51 , 12, -Math.PI))
//                                        .setReversed(true)
//                                        .waitSeconds(1)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,29),0)
//                                        .resetConstraints()
//                                        .strafeRight(12)
//                                        .setReversed(false)
//                                        //round1
//                                        .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .lineToConstantHeading(new Vector2d(-51, 12))
//                                        .resetConstraints()
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,29),0)
//                                        //parking
//                                        .lineToSplineHeading(new Pose2d(50, 12, -Math.PI/2))
//                                        .lineToConstantHeading(new Vector2d(60, 12))
//                                        .setReversed(false)
//                                        .resetConstraints()
//                                        .waitSeconds(1)

                                //center 50 pts
                                .lineToSplineHeading(new Pose2d(-51,-24, 0))
                                .lineToConstantHeading(new Vector2d(-55, -18))
                                .lineToSplineHeading(new Pose2d(-34 , -12, -Math.PI))
                                .lineToSplineHeading(new Pose2d(-51 , -12, -Math.PI))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-34,-12),0)
                                .splineToConstantHeading(new Vector2d(28,-12),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(50,-38),0)
                                .strafeLeft(12)
                                .setReversed(false)
                                .resetConstraints()
                                //round1
                                .splineToConstantHeading(new Vector2d(28, -12), -Math.PI)
                                .splineToConstantHeading(new Vector2d(-34, -12), -Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                                .lineToConstantHeading(new Vector2d(-51, -12))
                                .resetConstraints()
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-34,-12),0)
                                .splineToConstantHeading(new Vector2d(28,-12),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(50,-29),0)
                                //parking
                                .lineToSplineHeading(new Pose2d(50, -12, -Math.PI/2))
                                .lineToConstantHeading(new Vector2d(60, -12))
                                .resetConstraints()
                                .setReversed(false)

                                //left
//                                        .lineToSplineHeading(new Pose2d(-44,30, 0))
//                                        .lineToSplineHeading(new Pose2d(-44 , 12, -Math.PI))
//                                        .lineToSplineHeading(new Pose2d(-51 , 12, -Math.PI))
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,48),0) //52
//                                        .strafeLeft(15)
//                                        .setReversed(false)
//                                        .resetConstraints()
//                                        //round1
//                                        .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .lineToConstantHeading(new Vector2d(-51, 12))
//                                        .resetConstraints()
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,29),0)
//                                        .lineToSplineHeading(new Pose2d(50, 12, -Math.PI/2))
//                                        .lineToConstantHeading(new Vector2d(60, 12))
//                                        .resetConstraints()
//                                        .setReversed(false)

                                .build()
                );

        RoadRunnerBotEntity BlueAuto = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.4)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-39, 64, 0))

                                        //right 50pts
//                                        .lineToSplineHeading(new Pose2d(-35,32, -Math.PI))
//                                        .lineToSplineHeading(new Pose2d(-32 , 12, -Math.PI))
//                                        .lineToSplineHeading(new Pose2d(-51 , 12, -Math.PI))
//                                        .setReversed(true)
//                                        .waitSeconds(1)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,29),0)
//                                        .resetConstraints()
//                                        .strafeRight(12)
//                                        .setReversed(false)
//                                        //round1
//                                        .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .lineToConstantHeading(new Vector2d(-51, 12))
//                                        .resetConstraints()
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,29),0)
//                                        //parking
//                                        .lineToSplineHeading(new Pose2d(50, 12, -Math.PI/2))
//                                        .lineToConstantHeading(new Vector2d(60, 12))
//                                        .setReversed(false)
//                                        .resetConstraints()
//                                        .waitSeconds(1)

                                        //center 50 pts
                                        .lineToSplineHeading(new Pose2d(-51,24, 0))
                                        .lineToSplineHeading(new Pose2d(-51 , 12, -Math.PI))
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-34,12),0)
                                        .splineToConstantHeading(new Vector2d(28,12),0)
                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                                        .splineToConstantHeading(new Vector2d(50,38),0)
                                        .strafeLeft(12)
                                        .setReversed(false)
                                        .resetConstraints()
                                        //round1
                                        .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
                                        .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                                        .lineToConstantHeading(new Vector2d(-51, 12))
                                        .resetConstraints()
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-34,12),0)
                                        .splineToConstantHeading(new Vector2d(28,12),0)
                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
                                        .splineToConstantHeading(new Vector2d(50,29),0)
                                        //parking
                                        .lineToSplineHeading(new Pose2d(50, 12, -Math.PI/2))
                                        .lineToConstantHeading(new Vector2d(60, 12))
                                        .resetConstraints()
                                        .setReversed(false)

                                        //left
//                                        .lineToSplineHeading(new Pose2d(-44,30, 0))
//                                        .lineToSplineHeading(new Pose2d(-44 , 12, -Math.PI))
//                                        .lineToSplineHeading(new Pose2d(-51 , 12, -Math.PI))
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,48),0) //52
//                                        .strafeLeft(15)
//                                        .setReversed(false)
//                                        .resetConstraints()
//                                        //round1
//                                        .splineToConstantHeading(new Vector2d(28, 12), -Math.PI)
//                                        .splineToConstantHeading(new Vector2d(-34, 12), -Math.PI)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .lineToConstantHeading(new Vector2d(-51, 12))
//                                        .resetConstraints()
//                                        .setReversed(true)
//                                        .splineToConstantHeading(new Vector2d(-34,12),0)
//                                        .splineToConstantHeading(new Vector2d(28,12),0)
//                                        .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 12.4), SampleMecanumDrive.getAccelerationConstraint(35))
//                                        .splineToConstantHeading(new Vector2d(50,29),0)
//                                        .lineToSplineHeading(new Pose2d(50, 12, -Math.PI/2))
//                                        .lineToConstantHeading(new Vector2d(60, 12))
//                                        .resetConstraints()
//                                        .setReversed(false)

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueAuto)
                .start();
    }
}