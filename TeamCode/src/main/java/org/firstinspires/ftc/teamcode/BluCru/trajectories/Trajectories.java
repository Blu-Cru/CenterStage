package org.firstinspires.ftc.teamcode.BluCru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.states.Side;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {
    private static Pose2d closeStartingPose;
    private static Pose2d closePlacementFarPose;
    private static Pose2d closePlacementClosePose;
    private static Pose2d closePlacementCenterPose;

    private static Pose2d farStartingPose;
    private static Pose2d farPlacementFarPose;
    private static Pose2d farPlacementClosePose;
    private static Pose2d farPlacementCenterPose;

    private static final Pose2d redRightStartingPose = new Pose2d(13.5, -63, Math.toRadians(-90));
    private static final Pose2d redRightPlacementLeftPose = new Pose2d(5, -38, Math.toRadians(-45));
    private static final Pose2d redRightPlacementRightPose = new Pose2d(17, -40, Math.toRadians(-135));
    private static final Pose2d redRightPlacementCenterPose = new Pose2d(13.5, -30, Math.toRadians(-90));

    private static final Pose2d redLeftStartingPose = new Pose2d(-34.5, -63, Math.toRadians(-90));
    private static final Pose2d redLeftPlacementLeftPose = new Pose2d(-43, -38, Math.toRadians(-45));

    private static final Pose2d blueRightStartingPose = new Pose2d(-37.5, 63, Math.toRadians(90));
    private static final Pose2d blueRightPlacementLeftPose = new Pose2d(-29, 38, Math.toRadians(135));

    private static final Pose2d blueLeftStartingPose = new Pose2d(10.5, 63, Math.toRadians(90));
    private static final Pose2d blueLeftPlacementLeftPose = new Pose2d(19, 38, Math.toRadians(135));

    Side side;
    double reflect;

    public Trajectories(Alliance alliance, Side side) {
        if(alliance == Alliance.RED) {
            reflect = 1.0;
        } else {
            reflect = -1.0;
        }
        this.side = side;

        closeStartingPose = new Pose2d(12, -63 * reflect, Math.toRadians(-90 * reflect));
        closePlacementFarPose = new Pose2d(5, -38 * reflect, Math.toRadians(-45 * reflect));
        closePlacementClosePose = new Pose2d(17, -40 * reflect, Math.toRadians(-135 * reflect));
        closePlacementCenterPose = new Pose2d(13.5, -30 * reflect, Math.toRadians(-90 * reflect));

        farStartingPose = new Pose2d(-36, -63 * reflect, Math.toRadians(-90 * reflect));
        farPlacementFarPose = new Pose2d(-43, -38 * reflect, Math.toRadians(-45 * reflect));
        farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
        farPlacementCenterPose = new Pose2d(-34.5, -30 * reflect, Math.toRadians(-90 * reflect));
    }

    public TrajectorySequence placementFar(Robot robot) {
        TrajectorySequence sequence = null;
        switch (side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementFarPose, Math.toRadians(135 * reflect))
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(-36, -50*reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(farPlacementFarPose, Math.toRadians(135 * reflect))
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence placementClose(Robot robot) {
        TrajectorySequence sequence = null;
        switch (side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementClosePose, Math.toRadians(45 * reflect))
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(-36, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(farPlacementClosePose, Math.toRadians(45 * reflect))
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence placementCenter(Robot robot) {
        TrajectorySequence sequence = null;
        switch (side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementCenterPose, Math.toRadians(90 * reflect))
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(-36, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(farPlacementCenterPose, Math.toRadians(90 * reflect))
                        .build();
                break;
        }
        return sequence;
    }
}
