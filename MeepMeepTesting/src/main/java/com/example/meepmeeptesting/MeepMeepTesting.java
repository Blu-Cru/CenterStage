package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double reflect = 1.0;

    static Pose2d closeStartingPose = new Pose2d(12, -63 * reflect, Math.toRadians(-90 * reflect));
    static Pose2d closePlacementFarPose = new Pose2d(4.5, -39 * reflect, Math.toRadians(-45 * reflect));
    static Pose2d closePlacementClosePose = new Pose2d(17, -40 * reflect, Math.toRadians(-135 * reflect));
    static Pose2d closePlacementCenterPose = new Pose2d(15, -30 * reflect, Math.toRadians(-90 * reflect));

    static Pose2d farStartingPose = new Pose2d(-36, -63 * reflect, Math.toRadians(-90 * reflect));
    static Pose2d farPlacementFarPose = new Pose2d(-43.5, -39 * reflect, Math.toRadians(-45 * reflect));
    static Pose2d farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
    static Pose2d farPlacementCenterPose = new Pose2d(-33, -30 * reflect, Math.toRadians(-90 * reflect));


    static Pose2d squarePose = new Pose2d(40, -36 * reflect, Math.toRadians(180));

    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(180), 16.9);


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(closePlacementCenterPose)
                                .setTangent(Math.toRadians(-90 * reflect))
                                .splineToLinearHeading(squarePose, 0)
                                .build()
                );

        RoadRunnerBotEntity swervePlacementLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13.5, -63, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(13.5, -52), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(5, -38, Math.toRadians(-45)), Math.toRadians(135))
                                .build()
                );

        RoadRunnerBotEntity swervePlacementRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34.5, -63, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-34.5, -53), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-31, -40, Math.toRadians(-135)), Math.toRadians(45))
                                .build()
                );

        RoadRunnerBotEntity depositScanFromPlacementLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(8, -38, Math.toRadians(-30)))
                                .splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity depositScanFromPlacementCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, -30, Math.toRadians(-120)))
                                .splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity depositCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40, -35, Math.toRadians(180)))
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(60, -35), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(test)
                .start();
    }
}