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

    private static Pose2d depositFarPose;
    private static Pose2d depositCenterPose;
    private static Pose2d depositClosePose;

    private static Pose2d parkPose;

    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(14, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);

    Side side;
    double reflect;

    public Trajectories(Alliance alliance, Side side) {
        if(alliance == Alliance.RED) {
            reflect = 1.0;
        } else {
            reflect = -1.0;
        }
        this.side = side;

        closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
        closePlacementFarPose = new Pose2d(4.5, -39 * reflect, Math.toRadians(-45 * reflect));
        closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
        closePlacementCenterPose = new Pose2d(15, -32 * reflect, Math.toRadians(-90 * reflect));

        farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
        farPlacementFarPose = new Pose2d(-43.5, -39 * reflect, Math.toRadians(-45 * reflect));
        farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
        farPlacementCenterPose = new Pose2d(-33, -32 * reflect, Math.toRadians(-90 * reflect));

        squarePose = new Pose2d(45, -36 * reflect, Math.toRadians(180));

        depositFarPose = new Pose2d(50, -30 * reflect, Math.toRadians(180));
        depositCenterPose = new Pose2d(50, -36 * reflect, Math.toRadians(180));
        depositClosePose = new Pose2d(50, -42 * reflect, Math.toRadians(180));

        parkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));
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
                        .splineToLinearHeading(squarePose,0)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementFarPose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(-45 * reflect))
                        .splineToLinearHeading(new Pose2d(-36, -45*reflect, Math.toRadians(-90*reflect)), Math.toRadians(-45 * reflect))
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(-36, -18*reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(new Pose2d(-30, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(25, -10*reflect), Math.toRadians(0))
                        .splineToConstantHeading(squarePose.vec(), Math.toRadians(0))
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
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(-90 * reflect))
                        .splineToConstantHeading(new Vector2d(-55, -36*reflect), Math.toRadians(90 * reflect))
                        .splineTo(new Vector2d(-55, -24*reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(new Pose2d(-45, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                        .setVelConstraint(normalVelocity)
                        .splineToConstantHeading(new Vector2d(25, -10*reflect), Math.toRadians(0))
                        .splineToConstantHeading(squarePose.vec(), Math.toRadians(0))
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
                        .setTangent(Math.toRadians(-90 * reflect))
                        .splineToSplineHeading(new Pose2d(28, -50 * reflect, Math.toRadians(180)), 0)
                        .splineToConstantHeading(squarePose.vec(), 0)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementClosePose)
                        .setVelConstraint(normalVelocity)
                        .splineToConstantHeading(new Vector2d(-45, -35*reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(new Pose2d(-32, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(25, -10*reflect), Math.toRadians(0))
                        .splineToConstantHeading(squarePose.vec(), Math.toRadians(0))
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence depositFar(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(squarePose)
                .setVelConstraint(slowVelocity)
                .setTangent(Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderAutoPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.intake.toggleWrist();
                })
                .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakePower;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,() -> {
                    robot.intake.toggleWrist();
                    robot.intake.outtakeRollersPower = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                    robot.lift.resetLiftStallTimer();
                    robot.lift.liftState = LiftState.RETRACT;
                    robot.lift.setTargetPos(0);
                })
                .waitSeconds(2.5)

                .build();
        return sequence;
    }

    public TrajectorySequence depositCenter(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(squarePose)
                .setVelConstraint(slowVelocity)
                .setTangent(Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderAutoPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.intake.toggleWrist();
                })
                .splineToConstantHeading(depositCenterPose.vec(), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,() -> {
                    robot.intake.toggleWrist();
                    robot.intake.outtakeRollersPower = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                    robot.lift.resetLiftStallTimer();
                    robot.lift.liftState = LiftState.RETRACT;
                    robot.lift.setTargetPos(0);
                })
                .waitSeconds(2.5)
                .build();
        return sequence;
    }

    public TrajectorySequence depositClose(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(squarePose)
                .setVelConstraint(slowVelocity)
                .setTangent(Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.liftState = LiftState.AUTO;
                    robot.lift.setTargetPos(Constants.sliderAutoPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.intake.toggleWrist();
                })
                .splineToConstantHeading(depositClosePose.vec(), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,() -> {
                    robot.intake.toggleWrist();
                    robot.intake.outtakeRollersPower = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                    robot.lift.resetLiftStallTimer();
                    robot.lift.liftState = LiftState.RETRACT;
                    robot.lift.setTargetPos(0);
                })
                .waitSeconds(2.5)
                .build();
        return sequence;
    }

    public TrajectorySequence parkFar(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(depositFarPose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36, -18*reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(parkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkCenter(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(depositClosePose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(45, -18*reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(parkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkClose(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(depositClosePose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(45, -18*reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(parkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkFarFromFar(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementFarPose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToLinearHeading(new Pose2d(-36, -45*reflect, Math.toRadians(-90*reflect)), Math.toRadians(-45 * reflect))
                .setTangent(Math.toRadians(90 * reflect))
                .splineToConstantHeading(new Vector2d(-36, -18*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(new Pose2d(-30, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                .setVelConstraint(normalVelocity)
                .splineToConstantHeading(parkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkCloseFromFar(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementClosePose)
                .setVelConstraint(normalVelocity)
                .splineToConstantHeading(new Vector2d(-45, -35*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(new Pose2d(-32, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(25, -10*reflect), Math.toRadians(0))
                .setVelConstraint(normalVelocity)
                .splineToConstantHeading(parkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkCenterFromFar(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementCenterPose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(-90 * reflect))
                .splineToConstantHeading(new Vector2d(-55, -36*reflect), Math.toRadians(90 * reflect))
                .splineTo(new Vector2d(-55, -24*reflect), Math.toRadians(90 * reflect))
                .splineToSplineHeading(new Pose2d(-45, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                .setVelConstraint(fastVelocity)
                .splineToConstantHeading(parkPose.vec(), Math.toRadians(0))
                .build();
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
