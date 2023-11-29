package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity placementClose = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(10, 59))
                                .splineToSplineHeading(new Pose2d(10, 40, Math.toRadians(30)), Math.toRadians(-120))
                                .splineToLinearHeading(new Pose2d(10, 38, Math.toRadians(30)), Math.toRadians(-150))
                                .build()
                );

        RoadRunnerBotEntity placementClose2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(10, -59))
                                .splineToSplineHeading(new Pose2d(9, -38, Math.toRadians(-30)), Math.toRadians(150))
                                .build()
                );

        RoadRunnerBotEntity placementCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(10, -38))
                                .splineToSplineHeading(new Pose2d(13, -30, Math.toRadians(-120)), Math.toRadians(60))
                                .build()
                );

        RoadRunnerBotEntity placementRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -61, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(23, -40), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity depositScanFromPlacementLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(8, -38, Math.toRadians(-30)))
                                .splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity depositScanFromPlacementCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, -30, Math.toRadians(-120)))
                                .splineToSplineHeading(new Pose2d(40, -35, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity depositCenter = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40, -35, Math.toRadians(180)))
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(60, -35), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(placementClose)
                .addEntity(placementClose2)
                .start();
    }
}