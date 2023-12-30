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

    private static Pose2d closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
    private static Pose2d closePlacementFarPose = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
    private static Pose2d closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
    private static Pose2d closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

    private static Pose2d farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
    private static Pose2d farPlacementFarPose = new Pose2d(-44, -39 * reflect, Math.toRadians(-45 * reflect));
    private static Pose2d farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
    private static Pose2d farPlacementCenterPose = new Pose2d(-33, -31 * reflect, Math.toRadians(-90 * reflect));

    private static Pose2d squareFarPose = new Pose2d(50, -29 * reflect, Math.toRadians(180));
    private static Pose2d squareCenterPose = new Pose2d(50, -36 * reflect, Math.toRadians(180));
    private static Pose2d squareClosePose = new Pose2d(50, -43 * reflect, Math.toRadians(180));

    private static Pose2d depositFarPose = new Pose2d(52, -29 * reflect, Math.toRadians(180));
    private static Pose2d depositCenterPose = new Pose2d(52, -36 * reflect, Math.toRadians(180));
    private static Pose2d depositClosePose = new Pose2d(52, -43 * reflect, Math.toRadians(180));

    private static Pose2d alignClosePose = new Pose2d(-58, -36*reflect, Math.toRadians(180));

    private static Pose2d parkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));

    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(10, Math.toRadians(180), 14);
    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(28, 10, 14);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(40, 2, 14);



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(360), Math.toRadians(360), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(alignClosePose)
                                .setVelConstraint(normalVelocity)
                                .setTangent(Math.toRadians(60*reflect))
                                .splineToLinearHeading(new Pose2d(-46, -18*reflect, Math.toRadians(135*reflect)),0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}