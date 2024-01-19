package org.firstinspires.ftc.teamcode.blucru.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.blucru.Constants;
import org.firstinspires.ftc.teamcode.blucru.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.states.LiftState;
import org.firstinspires.ftc.teamcode.blucru.states.Side;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Intake;
import org.firstinspires.ftc.teamcode.blucru.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {
    static double reflect = 1;

    public static double stackSetupX;
    public static double stackX = -60;
    public static double depositX = 52;
    public static double depositSetupX = 45;

    public static Pose2d closeStartingPose;
    public static Pose2d closePlacementFarPose;
    public static Pose2d closePlacementClosePose;
    public static Pose2d closePlacementCenterPose;

    public static Pose2d farStartingPose;
    public static Pose2d farPlacementFarPose;
    public static Pose2d farPlacementClosePose;
    public static Pose2d farPlacementCenterPose;

//    public static Pose2d farPlacementClosePose = new Pose2d(-30, -18*reflect, Math.toRadians(135*reflect));

    public static Pose2d alignClosePose;

    public static Pose2d depositFarPose;
    public static Pose2d depositCenterPose;
    public static Pose2d depositClosePose;

    public static Pose2d closeParkPose;
    public static Pose2d farParkPose;

    private static TrajectoryVelocityConstraint normalVelocity = SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint slowVelocity = SampleMecanumDrive.getVelocityConstraint(14, Math.toRadians(180), DriveConstants.TRACK_WIDTH);
    private static TrajectoryVelocityConstraint fastVelocity = SampleMecanumDrive.getVelocityConstraint(40, 2.5, DriveConstants.TRACK_WIDTH);

    Side side;
    Placements placements;
    Poses poses;

    public Trajectories(Alliance alliance, Side side) {
        this.side = side;

        if(alliance == Alliance.RED) {
            reflect = 1;
        } else {
            reflect = -1;
        }

        closeStartingPose = new Pose2d(12, -62 * reflect, Math.toRadians(-90 * reflect));
        closePlacementFarPose = new Pose2d(5, -39 * reflect, Math.toRadians(-45 * reflect));
        closePlacementClosePose = new Pose2d(17.5, -40 * reflect, Math.toRadians(-135 * reflect));
        closePlacementCenterPose = new Pose2d(15, -31 * reflect, Math.toRadians(-90 * reflect));

        farStartingPose = new Pose2d(-36, -62 * reflect, Math.toRadians(-90 * reflect));
        farPlacementFarPose = new Pose2d(-44, -39 * reflect, Math.toRadians(-45 * reflect));
        farPlacementClosePose = new Pose2d(-31, -40 * reflect, Math.toRadians(-135 * reflect));

        alignClosePose = new Pose2d(-60, -36*reflect, Math.toRadians(180));

        depositFarPose = new Pose2d(depositX, -29 * reflect, Math.toRadians(180));
        depositCenterPose = new Pose2d(depositX, -36 * reflect, Math.toRadians(180));
        depositClosePose = new Pose2d(depositX, -43 * reflect, Math.toRadians(180));

        closeParkPose = new Pose2d(60, -60 * reflect, Math.toRadians(180));
        farParkPose = new Pose2d(60, -12 * reflect, Math.toRadians(180));
    }

    public TrajectorySequence farCenterCycle(Robot robot) {
        TrajectorySequence sequence = null;
        switch(side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
                    // set up placement
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineTo(new Vector2d(12, -55 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementFarPose, Math.toRadians(135 * reflect))
                    // placement
                        .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                            robot.purplePixelHolder.retracted = true;
                        })
                        .waitSeconds(0.5)
                    // set up deposit
                        .setTangent(-45*reflect)
                        .splineToConstantHeading(new Vector2d(8, -42 * reflect), 0)
                        .splineToConstantHeading(new Vector2d(10, -42*reflect), 0)
                        .splineToSplineHeading(new Pose2d(30, -35*reflect, Math.toRadians(180)), Math.toRadians(45*reflect))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29*reflect), 0)
                    // lift
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                    // deposit
                        .waitSeconds(1.2)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.unlock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.outtake.toggleWrist();
                        })
                    // retract
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(0);
                        })
                    // set up intake
                        .setVelConstraint(fastVelocity)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-53, -12*reflect), Math.toRadians(180))
                    // intake
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.setIntakeWristTargetAngle(Intake.WRIST_STACK1_DEG);
                            robot.intake.intakePower = 0.7;
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(stackX, -12*reflect), Math.toRadians(180))
                        .waitSeconds(1.5)
                    // intake finished, set up deposit
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                            robot.intake.intakePower = 0;
                            robot.intake.retractIntakeWrist();
                        })
                        .setTangent(0)
                        .setVelConstraint(fastVelocity)
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29*reflect), Math.toRadians(0))
                    // lift
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                    // deposit
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.unlock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(0);
                        })
                        .waitSeconds(1.2)
                    // set up intake
                        .setVelConstraint(fastVelocity)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-50, -12*reflect), Math.toRadians(180))
                    // intake
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.setIntakeWristTargetAngle(Intake.WRIST_STACK1_DEG);
                            robot.intake.intakePower = 0.7;
                        })
                    // intake finished, set up deposit
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                            robot.intake.intakePower = 0;
                            robot.intake.retractIntakeWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(stackX, -12*reflect), Math.toRadians(180))
                        .waitSeconds(1.5)
                        .setTangent(0)
                        .setVelConstraint(fastVelocity)
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29*reflect), Math.toRadians(0))
                    // lift
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                    // deposit
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.unlock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(0);
                        })
                        .waitSeconds(1.2)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                        // placement
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineTo(new Vector2d(-36, -55*reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(farPlacementFarPose, Math.toRadians(180))
                        .waitSeconds(1)
                        // set up intake
                        .setTangent(Math.toRadians(180))
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(stackX, -36 * reflect), Math.toRadians(180))
                        // intake
                        .waitSeconds(1)
                        // set up deposit
                        .setTangent(Math.toRadians(45 * reflect))
                        .setVelConstraint(normalVelocity)
                        .splineToConstantHeading(new Vector2d(-50, -12 * reflect), 0)
                        .setVelConstraint(fastVelocity)
                        .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29 * reflect), Math.toRadians(0))
                        // deposit
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                        .waitSeconds(1.2)
                            // drive to intake
                        .setVelConstraint(fastVelocity)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-50, -12*reflect), Math.toRadians(180))
                            // intake
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.setIntakeWristTargetAngle(Intake.WRIST_STACK3_DEG);
                            robot.intake.intakePower = 0.7;
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(stackX, -12*reflect), Math.toRadians(180))
                        .waitSeconds(1.5)
                            // lock and retract
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.retractIntakeWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                            robot.intake.intakePower = 0;
                            robot.outtake.lock();
                        })
                        .setTangent(0)
                        .setVelConstraint(fastVelocity)
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(45, -29*reflect), Math.toRadians(0))
                            // lift
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                            // release and retract
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.unlock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(0);
                        })
                        .waitSeconds(1.2)
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence centerCenterCycle(Robot robot) {
        TrajectorySequence sequence = null;
        switch (side) {
            case CLOSE:
                sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
                            // drive to placement
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(90 * reflect))
                        .splineToConstantHeading(new Vector2d(12, -50 * reflect), Math.toRadians(90 * reflect))
                        .splineToSplineHeading(closePlacementCenterPose, Math.toRadians(90 * reflect))
                        .waitSeconds(0.5)
                            // placement
                        .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                            robot.purplePixelHolder.retracted = true;
                        })
                            // drive to deposit
                        .setVelConstraint(normalVelocity)
                        .setTangent(Math.toRadians(-90 * reflect))
                        .splineToConstantHeading(new Vector2d(20, -36 * reflect), 0)
                        .splineToSplineHeading(new Pose2d(depositSetupX, -36*reflect, Math.toRadians(180)), 0)
                            // lift
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositCenterPose.vec(), Math.toRadians(0))
                            // release and retract
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.unlock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(0);
                        })
                        .waitSeconds(1.2)
                            // drive to intake
                        .setVelConstraint(fastVelocity)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-55, -12*reflect), Math.toRadians(180))
                            // intake
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.setIntakeWristTargetAngle(Intake.WRIST_STACK3_DEG);
                            robot.intake.intakePower = 0.7;
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(stackX, -12*reflect), Math.toRadians(180))
                        .waitSeconds(1.5)
                            // lock and retract
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.intake.retractIntakeWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {
                            robot.intake.intakePower = 0;
                            robot.outtake.lock();
                        })
                            // drive to deposit
                        .setTangent(0)
                        .setVelConstraint(fastVelocity)
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29*reflect), Math.toRadians(0))
                            // lift
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
                            // deposit and retract
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            robot.outtake.unlock();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                            robot.outtake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                            robot.outtake.lift.setMotionProfileTargetPos(0);
                        })
                        .waitSeconds(1.2)
                        .build();
                break;
            case FAR:
                sequence = robot.drivetrain.trajectorySequenceBuilder(farStartingPose)
                            // drive to placement center
                        .setVelConstraint(fastVelocity)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-36, -59*reflect), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-50, -24 * reflect, Math.toRadians(180)), Math.toRadians(90))
                            // placement

                        .waitSeconds(0.5)
                            // drive to intake middle
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-55, -24 * reflect), Math.toRadians(180))
                            // intake

                        .splineToConstantHeading(new Vector2d(stackX, -24 * reflect), Math.toRadians(180))
                        .waitSeconds(1.5)
                            // lock and retract

                            // drive to deposit far
                        .setVelConstraint(fastVelocity)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-50, -12 * reflect), 0)
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29*reflect), Math.toRadians(0))
                            // lift

                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(depositX, -29*reflect), Math.toRadians(0))
                            // deposit and retract

                        .waitSeconds(1.2)
                            // drive to intake far
                        .setVelConstraint(fastVelocity)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-55, -12*reflect), Math.toRadians(180))
                            // intake

                        .splineToConstantHeading(new Vector2d(stackX, -12*reflect), Math.toRadians(180))

                        .waitSeconds(1.5)
                            // lock and retract

                            // drive to deposit
                        .setVelConstraint(fastVelocity)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(30, -12*reflect), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(depositSetupX, -29*reflect), Math.toRadians(0))
                            // lift

                        .setVelConstraint(slowVelocity)
                        .splineToConstantHeading(new Vector2d(depositX, -29*reflect), Math.toRadians(0))
                            // deposit and retract

                        .waitSeconds(1.2)
                        .build();
                break;
        }
        return sequence;
    }

    public TrajectorySequence closeCenterCycle(Robot robot) {
        TrajectorySequence sequence = null;
            switch(side) {
                case CLOSE:
                    sequence = robot.drivetrain.trajectorySequenceBuilder(closeStartingPose)
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
                            .build();
                    break;
                case FAR:
                    break;
            }
        return sequence;
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
                            robot.outtake.lift.liftState = LiftState.AUTO;
                            robot.outtake.lift.setTargetPos(Constants.sliderAutoPos);
                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                            robot.intake.toggleWrist();
//                        })
                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
////                            robot.lift.liftState = LiftState.AUTO;
////                            robot.lift.setTargetPos(Constants.sliderLowPos);
//                            robot.intake.outtakeRollersPower = 0;
//                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
//                            robot.intake.toggleWrist();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
//                            robot.lift.setTargetPos(0);
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
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderAutoPos);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .splineToConstantHeading(depositFarPose.vec(), Math.toRadians(0))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderLowPos);
//                            robot.intake.outtakeRollersPower = 0;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
//                            robot.lift.setTargetPos(0);
//                        })
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
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderAutoPos);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .splineToConstantHeading(depositCenterPose.vec(), Math.toRadians(0))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderLowPos);
//                            robot.intake.outtakeRollersPower = 0;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
//                            robot.lift.setTargetPos(0);
//                        })
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
//                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                        robot.lift.liftState = LiftState.AUTO;
//                        robot.lift.setTargetPos(Constants.sliderAutoPos);
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                        robot.intake.toggleWrist();
//                    })
//                    .splineToConstantHeading(depositCenterPose.vec(), Math.toRadians(0))
//                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                        robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                        robot.lift.liftState = LiftState.AUTO;
//                        robot.lift.setTargetPos(Constants.sliderLowPos);
//                        robot.intake.outtakeRollersPower = 0;
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
//                        robot.intake.toggleWrist();
//                    })
//                    .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
//                        robot.lift.setTargetPos(0);
//                    })
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
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderAutoPos);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .splineToConstantHeading(depositClosePose.vec(), Math.toRadians(0))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderLowPos);
//                            robot.intake.outtakeRollersPower = 0;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
//                            robot.lift.setTargetPos(0);
//                        })
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
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderAutoPos);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .splineToConstantHeading(depositClosePose.vec(), Math.toRadians(0))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            robot.intake.outtakeRollersPower = Constants.outtakeRollersOuttakeAutoPower;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                            robot.lift.liftState = LiftState.AUTO;
//                            robot.lift.setTargetPos(Constants.sliderLowPos);
//                            robot.intake.outtakeRollersPower = 0;
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
//                            robot.intake.toggleWrist();
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.1,() -> {
//                            robot.lift.setTargetPos(0);
//                        })
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
