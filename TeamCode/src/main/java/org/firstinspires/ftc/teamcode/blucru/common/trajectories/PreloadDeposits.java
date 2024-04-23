package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class PreloadDeposits {
    public static double LIFT_TIME = -1;
    public static double WRIST_EXTEND_TIME = -0.65;
    public static double TURRET_TURN_TIME = -0.35;
    public static double INTAKE_1_TIME = 0.7;
//    public static double INTAKE_2_TIME = 0.5;
    public static double RELEASE_TIME = 0;
    public static double LIFT_CLEAR_AFTER_RELEASE_TIME = 0.8;
    public static double TOTAL_CLOSE_DEPOSIT_TIME = 0.8;

    public static double TOTAL_FAR_DEPOSIT_TIME = 1.5;

    double reflect;
    public PreloadDeposits(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence depositThroughCenterFromWingCenterFarStack(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(new Pose2d(Poses.STACK_SETUP_X, -12 * reflect, Math.toRadians(180)))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(0 * reflect))
                // reverse intake, lock
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.intakeWrist.dropToAutoMidPos();
                    robot.intake.setIntakePower(-1);
                    robot.outtake.lock();
                    robot.intake.stopReadingColor();
                })
                // stop intake, retract wrist
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })

                .splineToConstantHeading(new Vector2d(-10 + Poses.FIELD_OFFSET_X, -12 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(20, Poses.CENTER_Y * reflect), Math.toRadians(0))
//                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.setTargetPixelHeight(-1);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 - 45 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
//                .addTemporalMarker(() -> robot.drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.outtake.setTargetPixelHeight(0.5);
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.outtake.setTurretAngle(270);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    robot.outtake.setTargetPixelHeight(-1);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.outtake.unlock();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.incrementTargetHeight(1);
                    robot.outtake.resetLock();
                })
                .waitSeconds(1)
                .build();
    }

    public TrajectorySequence depositFromBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CLOSE_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(270 * reflect))
                .splineToConstantHeading(new Vector2d(30, -44 * reflect), 0)
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect, Math.toRadians(180)), 0)

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 45 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), 0)

                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME + LIFT_CLEAR_AFTER_RELEASE_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                    robot.outtake.centerTurret();
                })

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositFromBackdropCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(-90 * reflect))
                .splineToConstantHeading(new Vector2d(20, -40 * reflect), 0)
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, -36*reflect, Math.toRadians(180)), 0)

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                .addTemporalMarker(() -> robot.drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE))
                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME + LIFT_CLEAR_AFTER_RELEASE_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .addTemporalMarker(() -> robot.drivetrain.idle())
                .build();
    }

    public TrajectorySequence depositFromBackdropFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_FAR_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(-45*reflect)
                .splineToConstantHeading(new Vector2d(8, -42 * reflect), 0)
                .splineToConstantHeading(new Vector2d(10, -42*reflect), 0)
                .splineToSplineHeading(new Pose2d(30, -35 * reflect, Math.toRadians(180)), Math.toRadians(45*reflect))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y *reflect), 0)

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 - 45 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))

                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME + LIFT_CLEAR_AFTER_RELEASE_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
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
                    robot.intake.dropToStack(3);
                    robot.intake.setIntakePower(1);
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(-53 + Poses.FIELD_OFFSET_X, -24*reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_1_TIME)
                // lock and start outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.intakeWrist.dropToAutoMidPos();
                    robot.intake.setIntakePower(-0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-40 + Poses.FIELD_OFFSET_X, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))
                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(850);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 - 50 * reflect);
                })
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.outtake.setTurretAngle(270 + 30 * reflect);
                    robot.outtake.lift.setMotionProfileTargetPos(700);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.outtake.unlock();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(180 * reflect))
                // drop to intake, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.dropToStack(3);
                    robot.intake.setIntakePower(1);
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(-53 + Poses.FIELD_OFFSET_X, -24*reflect), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.intake.dropToStack(0);
//                })
//                .lineTo(new Vector2d(Poses.STACK_SETUP_X, -24 * reflect))
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.intakeWrist.dropToAutoMidPos();
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(45 * reflect))
                .splineToConstantHeading(new Vector2d(-45 + Poses.FIELD_OFFSET_X, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(670);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 - 50 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(900);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.lockFront();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.outtake.setTurretAngle(270 + 25 * reflect);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(670);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.unlock();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughCenterFromWingFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_CENTER_POSE)
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(150 * reflect))
                // drop down, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.dropToStack(3);
                    robot.intake.setIntakePower(1);
                    robot.outtake.unlock();
                })
                .splineToLinearHeading(new Pose2d(Poses.STACK_X, -12 * reflect, Math.toRadians(180)), Math.toRadians(100 * reflect))
                .waitSeconds(INTAKE_1_TIME)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.intake.dropToStack(0);
//                })
//                .lineTo(new Vector2d(Poses.STACK_SETUP_X, -12 * reflect))
//                .waitSeconds(INTAKE_2_TIME)
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                    robot.intake.intakeWrist.dropToAutoMidPos();
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .setTangent(Math.toRadians(0 * reflect))
                .splineToConstantHeading(new Vector2d(-45 + Poses.FIELD_OFFSET_X, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(30, Poses.CENTER_Y * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 60 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.outtake.setTurretAngle(270);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CLOSE_FOR_PERIM_POSE)
                .setVelConstraint(Constraints.FAST_VEL)
                .setTangent(Math.toRadians(180 * reflect))
                .splineToSplineHeading(new Pose2d(Poses.STACK_SETUP_X, -36*reflect, Math.toRadians(180)), Math.toRadians(180))
                .setVelConstraint(Constraints.NORMAL_VEL)
                // drop down, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.dropToStack(4);
                    robot.intake.setIntakePower(1);
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_1_TIME)
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 - 50 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.outtake.setTurretAngle(270 + 20 * reflect);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(225 * reflect))
                .splineToConstantHeading(new Vector2d(Poses.STACK_SETUP_X, -24 * reflect), Math.toRadians(180))
                .setVelConstraint(Constraints.NORMAL_VEL)
                // drop down, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.dropToStack(4);
                    robot.intake.setIntakePower(1);
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -24 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_1_TIME)
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .setVelConstraint(Constraints.FAST_VEL)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect), Math.toRadians(0))
                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 20 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), Math.toRadians(0))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.outtake.setTurretAngle(270 - 50 * reflect);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositThroughPerimeterFromWingFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.WING_PLACEMENT_FAR_FOR_PERIM_POSE)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(180 * reflect))
                .setVelConstraint(Constraints.NORMAL_VEL)
                // drop down, start intake, unlock
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.dropToStack(4);
                    robot.intake.setIntakePower(1);
                    robot.outtake.unlock();
                })
                .splineToConstantHeading(new Vector2d(Poses.STACK_X, -36 * reflect), Math.toRadians(180))
                .waitSeconds(INTAKE_1_TIME)
                // lock and start outtaking
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.lock();
                    robot.intake.setIntakePower(-0.7);
                })
                // stop outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .setTangent(Math.toRadians(-45 * reflect))
                .splineToConstantHeading(new Vector2d(-45 + Poses.FIELD_OFFSET_X, -60 * reflect), Math.toRadians(0))
                .setConstraints(Constraints.NORMAL_VEL, Constraints.NORMAL_ACCEL)
                .splineToConstantHeading(new Vector2d(30, -60 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, -36 * reflect), Math.toRadians(0))
                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 20 * reflect);
                })
                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CENTER_POSE.vec(), Math.toRadians(0))
                // release white pixel
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlockFrontLockBack();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.setTurretAngle(270 - 50 * reflect);
                })
                // release yellow pixel
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                })
                .waitSeconds(TOTAL_FAR_DEPOSIT_TIME)
                .build();
    }

    public TrajectorySequence depositCloseFromStart(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setTangent(Math.toRadians(90))
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .splineToConstantHeading(new Vector2d(13, -58 * reflect), Math.toRadians(60 * reflect))
                .splineToSplineHeading(new Pose2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_CLOSE_Y * reflect, Math.toRadians(180)), 0)

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.YELLOW_POS);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURRET_TURN_TIME, () -> {
                    robot.outtake.setTurretAngle(270 + 45 * reflect);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_CLOSE_POSE.vec(), 0)

                // release
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME, () -> {
                    robot.outtake.unlock();
                })
                // lift clear
                .UNSTABLE_addTemporalMarkerOffset(RELEASE_TIME + LIFT_CLEAR_AFTER_RELEASE_TIME, () -> {
                    robot.outtake.lift.setMotionProfileTargetPos(Lift.CLEAR_POS);
                    robot.outtake.centerTurret();
                })

                .waitSeconds(TOTAL_CLOSE_DEPOSIT_TIME)
                .build();
    }
}
