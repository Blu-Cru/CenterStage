package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static double reflect = 1;

    private static double stackSetupX = -55;
    private static double stackX = -61;
    private static double depositX = 52;
    private static double depositSetupX = 45;

    private static Pose2d closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
    private static Pose2d closePlacementFarPose = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
    private static Pose2d closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
    private static Pose2d closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

    private static Pose2d farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
    private static Pose2d farPlacementFarPose = new Pose2d(-56, -36 * reflect, Math.toRadians(180));
    private static Pose2d farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
    private static Pose2d farPlacementCenterPose = new Pose2d(-33, -31 * reflect, Math.toRadians(-90 * reflect));

    private static Pose2d squareFarPose = new Pose2d(50, -29 * reflect, Math.toRadians(180));
    private static Pose2d squareCenterPose = new Pose2d(50, -36 * reflect, Math.toRadians(180));
    private static Pose2d squareClosePose = new Pose2d(50, -43 * reflect, Math.toRadians(180));

    private static Pose2d depositFarPose = new Pose2d(depositX, -29 * reflect, Math.toRadians(180));
    private static Pose2d depositCenterPose = new Pose2d(depositX, -36 * reflect, Math.toRadians(180));
    private static Pose2d depositClosePose = new Pose2d(depositX, -43 * reflect, Math.toRadians(180));

    private static Pose2d alignClosePose = new Pose2d(-58, -36*reflect, Math.toRadians(180));

    private static Pose2d parkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));

    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 14);
    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(28, 10, 14);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(300), 14);



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(360), Math.toRadians(360), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(closeStartingPose)
                                    // drive to placement
                                .setVelConstraint(fastVelocity)
                                .setTangent(Math.toRadians(90 * reflect))
                                .lineTo(new Vector2d(24, -40 * reflect))
                                    // placement

                                .waitSeconds(0.5)
                                .setTangent(Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(30, -44 * reflect), 0)
                                .splineToSplineHeading(new Pose2d(depositSetupX, -43 * reflect, Math.toRadians(180)), 0)
                                    // lift

                                .splineToConstantHeading(new Vector2d(depositX, -43 * reflect), 0)
                                    // deposit and retract

                                .waitSeconds(1.2)

                                // drive to far stack
                                .setVelConstraint(fastVelocity)
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(stackSetupX, -12 * reflect), Math.toRadians(180))
                                // start intaking

                                .setVelConstraint(slowVelocity)
                                .splineToConstantHeading(new Vector2d(stackX, -12 * reflect), Math.toRadians(180))
                                .waitSeconds(1.5)
                                // lock and retract

                                // drive to deposit
                                .setTangent(0)
                                .setVelConstraint(fastVelocity)
                                .splineToConstantHeading(new Vector2d(30, -12 * reflect), 0)
                                .splineToConstantHeading(new Vector2d(depositSetupX, -29 * reflect), 0)
                                // lift

                                .splineToConstantHeading(new Vector2d(depositX, -29 * reflect), 0)
                                // deposit and retract

                                .waitSeconds(1.2)

                                // drive to far stack
                                .setVelConstraint(fastVelocity)
                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(stackSetupX, -12 * reflect), Math.toRadians(180))
                                // start intaking

                                .setVelConstraint(slowVelocity)
                                .splineToConstantHeading(new Vector2d(stackX, -12 * reflect), Math.toRadians(180))
                                .waitSeconds(1.5)
                                // lock and retract

                                // drive to deposit
                                .setTangent(0)
                                .setVelConstraint(fastVelocity)
                                .splineToConstantHeading(new Vector2d(30, -12 * reflect), 0)
                                .splineToConstantHeading(new Vector2d(depositSetupX, -29 * reflect), 0)
                                // lift

                                .splineToConstantHeading(new Vector2d(depositX, -29 * reflect), 0)
                                // deposit and retract

                                .waitSeconds(1.2)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}