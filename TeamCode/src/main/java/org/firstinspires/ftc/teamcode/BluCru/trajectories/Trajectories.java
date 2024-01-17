package org.firstinspires.ftc.teamcode.blucru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.states.Side;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {
    static double reflect = 1;

    public static Pose2d closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
    public static Pose2d closePlacementFarPose = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
    public static Pose2d closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
    public static Pose2d closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

    public static Pose2d farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
    public static Pose2d farPlacementFarPose = new Pose2d(-44, -39 * reflect, Math.toRadians(-45 * reflect));
    public static Pose2d farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));
    public static Pose2d farPlacementCenterPose = new Pose2d(-33, -31 * reflect, Math.toRadians(-90 * reflect));

//    public static Pose2d farPlacementClosePose = new Pose2d(-30, -18*reflect, Math.toRadians(135*reflect));

    public static Pose2d alignClosePose = new Pose2d(-60, -36*reflect, Math.toRadians(180));

    public static Pose2d depositFarPose = new Pose2d(52, -29 * reflect, Math.toRadians(180));
    public static Pose2d depositCenterPose = new Pose2d(52, -36 * reflect, Math.toRadians(180));
    public static Pose2d depositClosePose = new Pose2d(52, -43 * reflect, Math.toRadians(180));

    public static Pose2d closeParkPose = new Pose2d(60, -60 * reflect, Math.toRadians(180));
    public static Pose2d farParkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));

    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(14, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);

    Side side;
    Placements placements;
    Poses poses;

    public Trajectories(Alliance alliance, Side side) {
        this.side = side;
        this.poses = new Poses(alliance);

        this.placements = new Placements(side);
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

    public TrajectorySequence depositFar(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementFarPose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(-45*reflect)
                        .splineToConstantHeading(new Vector2d(8, -42 * reflect), 0)
                        .splineToConstantHeading(new Vector2d(10, -42*reflect), 0)
                        .splineToSplineHeading(new Pose2d(30, -35*reflect, Math.toRadians(180)), Math.toRadians(45*reflect))
                        .splineToConstantHeading(new Vector2d(45, -29*reflect), 0)
                        .setVelConstraint(slowVelocity)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderAutoPos);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.intake.toggleWrist();
                        })
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderLowPos);
                            robot.intake.outtakeRollersPower = 0;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                            robot.intake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
                            robot.lift.setTargetPos(0);
                        })
                        .waitSeconds(2)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementFarPose)
                    .setVelConstraint(normalVelocity)
                    .setTangent(Math.toRadians(-90 * reflect))
                    .splineToLinearHeading(new Pose2d(-36, -46*reflect, Math.toRadians(-90*reflect)), 0)
                    .setTangent(Math.toRadians(90 * reflect))
                    .splineToConstantHeading(new Vector2d(-36, -18*reflect), Math.toRadians(90 * reflect))
                    .splineToSplineHeading(new Pose2d(-30, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                    .setVelConstraint(fastVelocity)
                    .splineToConstantHeading(new Vector2d(25, -10*reflect), Math.toRadians(0))
                    .setVelConstraint(normalVelocity)
                    .splineToConstantHeading(new Vector2d(45, -29*reflect), Math.toRadians(0))
                        .setVelConstraint(slowVelocity)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderAutoPos);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.intake.toggleWrist();
                        })
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderLowPos);
                            robot.intake.outtakeRollersPower = 0;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                            robot.intake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
                            robot.lift.setTargetPos(0);
                        })
                        .waitSeconds(2)
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence depositCenter(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementCenterPose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(-90 * reflect))
                        .splineToConstantHeading(new Vector2d(20, -36 * reflect), 0)
                        .splineToSplineHeading(new Pose2d(45, -36*reflect, Math.toRadians(180)), 0)
                        .setVelConstraint(slowVelocity)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderAutoPos);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.intake.toggleWrist();
                        })
                        .splineToConstantHeading(depositCenterPose.vec(), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderLowPos);
                            robot.intake.outtakeRollersPower = 0;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                            robot.intake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
                            robot.lift.setTargetPos(0);
                        })
                        .waitSeconds(2)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementFarPose)
                    .setVelConstraint(normalVelocity)
                    .setTangent(Math.toRadians(-90 * reflect))
                    .splineToConstantHeading(new Vector2d(-55, -32*reflect), Math.toRadians(90 * reflect))
                    .splineTo(new Vector2d(-55, -24*reflect), Math.toRadians(90 * reflect))
                    .splineToSplineHeading(new Pose2d(-45, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                    .setVelConstraint(fastVelocity)
                    .splineToConstantHeading(new Vector2d(25, -10*reflect), Math.toRadians(0))
                    .setVelConstraint(normalVelocity)
                    .splineToConstantHeading(new Vector2d(45, -36 * reflect), Math.toRadians(0))
                    .setVelConstraint(slowVelocity)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.lift.liftState = LiftState.AUTO;
                        robot.lift.setTargetPos(Constants.sliderAutoPos);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        robot.intake.toggleWrist();
                    })
                    .splineToConstantHeading(depositCenterPose.vec(), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.lift.liftState = LiftState.AUTO;
                        robot.lift.setTargetPos(Constants.sliderLowPos);
                        robot.intake.outtakeRollersPower = 0;
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                        robot.intake.toggleWrist();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
                        robot.lift.setTargetPos(0);
                    })
                    .waitSeconds(2)
                    .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence depositClose(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closePlacementClosePose)
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(225 * reflect))
                        .splineToConstantHeading(new Vector2d(20, -52 * reflect), 0)
                        .splineToSplineHeading(new Pose2d(32, -52 * reflect, Math.toRadians(180)), 0)
                        .splineToConstantHeading(new Vector2d(45, -43*reflect), 0)
                        .setVelConstraint(slowVelocity)
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderAutoPos);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.intake.toggleWrist();
                        })
                        .splineToConstantHeading(depositClosePose.vec(), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderLowPos);
                            robot.intake.outtakeRollersPower = 0;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                            robot.intake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
                            robot.lift.setTargetPos(0);
                        })
                        .waitSeconds(2)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farPlacementClosePose)
                        .setVelConstraint(normalVelocity)
                        .splineToConstantHeading(new Vector2d(-45, -35*reflect), Math.toRadians(90 * reflect))
                        .setVelConstraint(fastVelocity)
                        .splineToSplineHeading(new Pose2d(-32, -10*reflect, Math.toRadians(180)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(30, -10*reflect), Math.toRadians(0))
                        .setVelConstraint(normalVelocity)
                        .splineToConstantHeading(new Vector2d(45, -43*reflect), Math.toRadians(0))
                        .setVelConstraint(slowVelocity)
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderAutoPos);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.intake.toggleWrist();
                        })
                        .splineToConstantHeading(depositClosePose.vec(), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.lift.liftState = LiftState.AUTO;
                            robot.lift.setTargetPos(Constants.sliderLowPos);
                            robot.intake.outtakeRollersPower = 0;
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                            robot.intake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
                            robot.lift.setTargetPos(0);
                        })
                        .waitSeconds(2)
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence alignCloseStack(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(90*reflect))
                .splineToConstantHeading(new Vector2d(-38, -50 * reflect), Math.toRadians(115*reflect))
                .splineToSplineHeading(new Pose2d(-55, -36 * reflect, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -36*reflect), Math.toRadians(180))
                .build();
        return sequence;
    }

    public TrajectorySequence stackPlacementClose(Robot robot) {
        TrajectorySequence sequence = null;

        sequence = robot.drivetrain.trajectorySequenceBuilder(alignClosePose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(45*reflect))
                .splineToConstantHeading(new Vector2d(-40, -18*reflect), 0)
                .splineToSplineHeading(new Pose2d(-30, -18*reflect, Math.toRadians(135*reflect)), 0)
                .build();
        return sequence;
    }

    public TrajectorySequence stackPlacementCenter(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(alignClosePose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(45*reflect))
                .setTangent(Math.toRadians(60*reflect))
                .splineToLinearHeading(new Pose2d(-46, -18*reflect, Math.toRadians(135*reflect)),0)

                .build();
        return sequence;
    }

    public TrajectorySequence parkFar(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(depositFarPose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36, -18*reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(farParkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkCenter(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(depositClosePose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(45, -18*reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(farParkPose.vec(), Math.toRadians(0))
                .build();
        return sequence;
    }

    public TrajectorySequence parkClose(Robot robot) {
        TrajectorySequence sequence = null;
        sequence = robot.drivetrain.trajectorySequenceBuilder(depositClosePose)
                .setVelConstraint(normalVelocity)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(45, -18*reflect), Math.toRadians(90 * reflect))
                .splineToConstantHeading(farParkPose.vec(), Math.toRadians(0))
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
                .splineToConstantHeading(farParkPose.vec(), Math.toRadians(0))
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
                .splineToConstantHeading(farParkPose.vec(), Math.toRadians(0))
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
                .splineToConstantHeading(farParkPose.vec(), Math.toRadians(0))
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
