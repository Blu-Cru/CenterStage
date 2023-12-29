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

    private static Pose2d parkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));

    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(180), 14);
    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(28, Math.toRadians(180), 14);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(40, 2, 14);



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity test = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(closePlacementClosePose)
                                .setVelConstraint(normalVelocity)
                                .setTangent(Math.toRadians(225 * reflect))
                                .splineToConstantHeading(new Vector2d(20, -52 * reflect), 0)
                                .splineToSplineHeading(new Pose2d(32, -52 * reflect, Math.toRadians(180)), 0)
                                .splineToConstantHeading(squareClosePose.vec(), 0)
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