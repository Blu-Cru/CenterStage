package org.firstinspires.ftc.teamcode.BluCru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.BluCru.Constants;
import org.firstinspires.ftc.teamcode.BluCru.states.Alliance;
import org.firstinspires.ftc.teamcode.BluCru.states.LiftState;
import org.firstinspires.ftc.teamcode.BluCru.states.Side;
import org.firstinspires.ftc.teamcode.BluCru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

    private static Pose2d squarePose;

    private static double slowTurnVelocity = 10;
    private static double slowTurnAccel = Math.toRadians(30);
    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(28, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(180), DriveConstants.TRACK_WIDTH);

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
        closePlacementFarPose = new Pose2d(4.5, -39 * reflect, Math.toRadians(-45 * reflect));
        closePlacementClosePose = new Pose2d(17, -40 * reflect, Math.toRadians(-135 * reflect));
        closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

        farStartingPose = new Pose2d(-36, -63 * reflect, Math.toRadians(-90 * reflect));
        farPlacementFarPose = new Pose2d(-43.5, -39 * reflect, Math.toRadians(-45 * reflect));
        farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
        farPlacementCenterPose = new Pose2d(-33, -30 * reflect, Math.toRadians(-90 * reflect));


        squarePose = new Pose2d(45, -36 * reflect, Math.toRadians(180));
    }

    public TrajectorySequence placementFar(Robot robot) {
        TrajectorySequence sequence = null;
        switch (side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
                        .setVelConstraint(slowVelocity)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineTo(new Vector2d(12, -55 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementFarPose, Math.toRadians(135 * reflect))
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        .setVelConstraint(slowVelocity)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineTo(new Vector2d(-36, -55*reflect), Math.toRadians(90 * reflect))
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
                        .setVelConstraint(slowVelocity)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementClosePose, Math.toRadians(45 * reflect))
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        .setVelConstraint(slowVelocity)
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
                        .setVelConstraint(slowVelocity)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementCenterPose, Math.toRadians(90 * reflect))
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        .setVelConstraint(slowVelocity)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(-36, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(farPlacementCenterPose, Math.toRadians(90 * reflect))
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence squareFar(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementFarPose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(-45*reflect)
                        .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderLowPos);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                            robot.intake.toggleWrist();
                        })
                        .splineToLinearHeading(squarePose,0)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementFarPose)
                        .setVelConstraint(normalVelocity)
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence squareCenter(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementCenterPose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(-90 * reflect))
                        .splineToLinearHeading(squarePose, 0)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementCenterPose)
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence squareClose(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementClosePose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(-135 * reflect))
                        .splineToLinearHeading(squarePose, 0)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementClosePose)
                        .build();
                break;
        }
        return sequence;
    }

    public Pose2d getStartPose() {
        switch (side) {
            case CLOSE:
                return closeStartingPose;
            case FAR:
                return farStartingPose;
        }
        return null;
    }
}
