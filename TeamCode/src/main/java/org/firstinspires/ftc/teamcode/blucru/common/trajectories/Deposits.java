package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Deposits {
    public static double WRIST_EXTEND_AFTER_LIFT_TIME = 0.2;
    public static double TURRET_TURN_AFTER_WRIST_TIME = 0.2;
    public static double RELEASE_TIME = 0.2;
    public static double LIFT_CLEAR_TIME = 0.5;
    public static double WRIST_RETRACT_TIME = 0.4;

    public static double TOTAL_CLOSE_DEPOSIT_TIME = 1;
    public static double INTAKE_TIME = 1.5;

    public static double reflect = 1;

    public Deposits(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence cyclePerimeterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                // lift up

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cyclePerimeterFromCenter(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cyclePerimeterFromClose(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -36 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(Math.toRadians(-45 * reflect))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(-30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromClose(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -12 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromCenter(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -12 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence cycleCenterFromFar(Robot robot, int stackHeight) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(135 * reflect))
                // retract turret

                // retract wrist

                // retract lift

                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect), Math.toRadians(180))
                // start and lower intake

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -12 * reflect), Math.toRadians(180))
                .waitSeconds(0.5)
                // lock and raise intake, start outtaking

                // stop outtaking

                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                // release pixel

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropClose(Robot robot) {
        double liftTime = 2; // time into trajectory to lift
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CLOSE_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(270 * reflect))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(liftTime + WRIST_EXTEND_AFTER_LIFT_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(liftTime + WRIST_EXTEND_AFTER_LIFT_TIME + TURRET_TURN_AFTER_WRIST_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 50 * reflect);
                })

                .splineToConstantHeading(new Vector2d(30, -44 * reflect), 0)
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect, Math.toRadians(180)), 0)

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), 0)

                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(LIFT_CLEAR_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                    robot.outtake.centerTurret();
                })

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropCenter(Robot robot) {
        double liftTime = 1.5; // time into trajectory to lift
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(-90 * reflect))
                // lift
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(liftTime + WRIST_EXTEND_AFTER_LIFT_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                .splineToConstantHeading(new Vector2d(20, -36 * reflect), 0)
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, -36*reflect, Math.toRadians(180)), 0)
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(LIFT_CLEAR_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropFar(Robot robot) {
        double liftTime = 2.5; // time into trajectory to lift
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_FAR_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(-45*reflect)

                // lift
                .UNSTABLE_addTemporalMarkerOffset(liftTime, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(liftTime + WRIST_EXTEND_AFTER_LIFT_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(liftTime + WRIST_EXTEND_AFTER_LIFT_TIME + TURRET_TURN_AFTER_WRIST_TIME, () -> {
                    robot.outtake.setTurretAngle(270 - 50 * reflect);
                })

                .splineToConstantHeading(new Vector2d(8, -42 * reflect), 0)
                .splineToConstantHeading(new Vector2d(10, -42*reflect), 0)
                .splineToSplineHeading(new Pose2d(30, -35 * reflect, Math.toRadians(180)), Math.toRadians(45*reflect))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y*reflect), 0)

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))

                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(LIFT_CLEAR_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                    robot.outtake.centerTurret();
                })
                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CLOSE_FOR_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(180 * reflect))
                // drop down, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.intake.dropToStack(4);
                    robot.intake.intakePower = 1;
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(-53, -24*reflect), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.intakePower = -1;
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.intakePower = 0;
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-40, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
//                // lift
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
//                })
//                // wrist back
//                .UNSTABLE_addTemporalMarkerOffset(1 + WRIST_EXTEND_AFTER_LIFT_TIME, () -> {
//                    robot.outtake.extendWrist();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(1 + WRIST_EXTEND_AFTER_LIFT_TIME + TURRET_TURN_AFTER_WRIST_TIME, () -> {
//                    robot.outtake.setTurretAngle(270 - 20 * reflect);
//                })
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
//                // release white pixel
//                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
//                    robot.outtake.lockBack();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.outtake.setTurretAngle(270 + 20 * reflect);
//                })
//                // release yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    robot.outtake.unlock();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
//                    robot.outtake.centerTurret();
//                })
                .waitSeconds(1.3)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(180 * reflect))
                // drop to intake, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.dropToStack(4);
                    robot.intake.intakePower = 1;
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(-53, -24*reflect), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.intakePower = -1;
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.intakePower = 0;
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
//                // lift
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
//                })
//                // wrist back
//                .UNSTABLE_addTemporalMarkerOffset(1 + WRIST_EXTEND_AFTER_LIFT_TIME, () -> {
//                    robot.outtake.extendWrist();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(1 + WRIST_EXTEND_AFTER_LIFT_TIME + TURRET_TURN_AFTER_WRIST_TIME, () -> {
//                    robot.outtake.setTurretAngle(270 - 20 * reflect);
//                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
//                // release white pixel
//                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
//                    robot.outtake.lockBack();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.outtake.setTurretAngle(270 + 20 * reflect);
//                })
//                // release yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    robot.outtake.unlock();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
//                    robot.outtake.centerTurret();
//                })
                .waitSeconds(1.3)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_CENTER_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(120 * reflect))
                // drop down, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.dropToStack(4);
                    robot.intake.intakePower = 1;
                    robot.outtake.unlock();
                })
                .splineToLinearHeading(new Pose2d(Poses.STACK_X, -12 * reflect, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.intakePower = -1;
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.intakePower = 0;
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(0 * reflect))
                .splineToConstantHeading(new Vector2d(-45, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
//                // lift
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
//                })
//                // wrist back
//                .UNSTABLE_addTemporalMarkerOffset(1 + WRIST_EXTEND_AFTER_LIFT_TIME, () -> {
//                    robot.outtake.extendWrist();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(1 + WRIST_EXTEND_AFTER_LIFT_TIME + TURRET_TURN_AFTER_WRIST_TIME, () -> {
//                    robot.outtake.setTurretAngle(270 - 20 * reflect);
//                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_FAR_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
//                // release white pixel
//                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
//                    robot.outtake.lockBack();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.outtake.setTurretAngle(270 + 20 * reflect);
//                })
//                // release yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
//                    robot.outtake.unlock();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
//                    robot.outtake.centerTurret();
//                })
                .waitSeconds(1.3)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CLOSE_FOR_PERIM_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(180 * reflect))
                .splineToSplineHeading(new Pose2d(Poses.STACK_SETUP_X, -36*reflect, Math.toRadians(180)), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(1.3)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -24 * reflect), Math.toRadians(180))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.BACKDROP_CLOSE_Y * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_PERIM_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(180 * reflect))
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                // drop down, start intake, unlock

                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_TIME)
                // lock and start outtaking

                // stop outtake

                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))
                // lift

                // wrist back

                // turn turret

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release white pixel

                // turn turret

                // release yellow pixel

                .waitSeconds(1.3)
                .build();
    }
}
