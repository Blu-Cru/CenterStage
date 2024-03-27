package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double reflect = 1;

    public static double DEPOSIT_TIME = 1.2;
    public static double INTAKE_TIME = 0.5;
    public static double RELEASE_TIME = 0.1;
    public static double WAIT_TIME = 0.1;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(360), Math.toRadians(400), 12.6)
                .setDimensions(14.3,14.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(Poses.STACK_SETUP_X, -12 * reflect, Math.toRadians(180)))
                                .setTangent(Math.toRadians(90 * reflect))
                                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                                .splineToConstantHeading(new Vector2d(-36 + Poses.FIELD_OFFSET_X, -58 * reflect), Math.toRadians(90 * reflect))
                                .splineToSplineHeading(new Pose2d(-45 + Poses.FIELD_OFFSET_X, -25 * reflect, Math.toRadians(180 * reflect)), Math.toRadians(135*reflect))
                                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                                    robot.purplePixelHolder.release(reflect);
//                                    robot.intake.dropToStack(4);
//                                })
                                .splineToConstantHeading(new Vector2d(-47 + Poses.FIELD_OFFSET_X, -23 * reflect), Math.toRadians(140*reflect))
                                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                                    robot.intake.intake();
//                                    robot.intakingInAuto = true;
//                                })
//                                .splineToConstantHeading(endPose.vec(), Math.toRadians(130*reflect))
//                                .addTemporalMarker(() -> robot.drivetrain.lockTo(endPose))
                                .waitSeconds(0.5)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}