package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double reflect = 1.0;

    public static Pose2d closeStartingPose = new Pose2d(12, -63 * reflect, Math.toRadians(-90 * reflect));
    public static Pose2d closePlacementFarPose = new Pose2d(5, -38 * reflect, Math.toRadians(-45 * reflect));
    public static Pose2d closePlacementClosePose = new Pose2d(17, -40 * reflect, Math.toRadians(-135 * reflect));
    public static Pose2d closePlacementCenterPose = new Pose2d(13.5, -30 * reflect, Math.toRadians(-90 * reflect));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(closeStartingPose)
                                .setTangent(Math.toRadians(90 * reflect))
                                .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                                .splineToSplineHeading(closePlacementCenterPose, Math.toRadians(90 * reflect))
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