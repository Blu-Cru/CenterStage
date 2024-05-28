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
                .setConstraints(40, 40, Math.toRadians(500), Math.toRadians(600), 12.6)
                .setDimensions(14.3,14.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                                .setVelConstraint(Constraints.FAST_VEL)
                                .setTangent(Math.toRadians(90 * reflect))
                                .splineToSplineHeading(new Pose2d(12, -50 * reflect, Math.toRadians(-45 * reflect)), Math.toRadians(90 * reflect))
//                                .splineToSplineHeading(Poses.BACKDROP_PLACEMENT_FAR_POSE, Math.toRadians(90 * reflect))
                                .splineToConstantHeading(new Pose2d(8, -40, Math.toRadians(-45 * reflect)).vec(), Math.toRadians(135 * reflect))
                                .splineToSplineHeading(new Pose2d(10, -30, Math.toRadians(20 * reflect)), Math.toRadians(30 * reflect))
                                // maybe stop trajectory here and PID to backboard?
                                .splineToSplineHeading(new Pose2d(45, -30, Math.toRadians(180 * reflect)), Math.toRadians(0 * reflect))
                                // release purple pixel
//                                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                                .waitSeconds(WAIT_TIME)
                                .build()
                );

        RoadRunnerBotEntity test1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(500), Math.toRadians(600), 12.6)
                .setDimensions(14.3,14.3)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                                        .setVelConstraint(Constraints.FAST_VEL)
                                        .setTangent(Math.toRadians(90 * reflect))
                                        .splineToSplineHeading(new Pose2d(12, -50 * reflect, Math.toRadians(-45 * reflect)), Math.toRadians(90 * reflect))
//                                .splineToSplineHeading(Poses.BACKDROP_PLACEMENT_FAR_POSE, Math.toRadians(90 * reflect))
                                        .splineToConstantHeading(new Pose2d(8, -40 * reflect, Math.toRadians(-45 * reflect)).vec(), Math.toRadians(135 * reflect))
                                        .setTangent(Math.toRadians(-10*reflect))
//                                        .splineToSplineHeading(new Pose2d(10, -30, Math.toRadians(20 * reflect)), Math.toRadians(30 * reflect))
                                        // maybe stop trajectory here and PID to backboard?
                                        .splineToLinearHeading(new Pose2d(45, -30 * reflect, Math.toRadians(180 * reflect)), Math.toRadians(0 * reflect))
                                        // release purple pixel
//                                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> robot.purplePixelHolder.retractRight())
                                        .waitSeconds(WAIT_TIME)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .addEntity(test1)
                .start();
    }
}