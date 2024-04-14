//package org.firstinspires.ftc.teamcode.blucru.common.trajectories;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//
//import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//public class DepositTrajectories {
//    public static double
//            LIFT_TIME = -1.0,
//            WRIST_EXTEND_TIME = -0.75,
//            TURN_TURRET_TIME = -0.4,
//            TOTAL_DEPOSIT_TIME = 0.2;
//    double reflect;
//
//    public DepositTrajectories (double reflect) {
//        this.reflect = reflect;
//    }
//
//    public TrajectorySequence depositCenterFromFarStack(Robot robot, double pixelHeight, double turretAngleDelta, Pose2d endPose) {
//        return robot.drivetrain.trajectorySequenceBuilder(new Pose2d(Poses.STACK_SETUP_X, -12 * reflect, Math.toRadians(180 * reflect)))
//                .setTangent(0)
//                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
//                .addTemporalMarker(() -> {
//                    robot.intakingInAuto = false;
//                    robot.intake.stopReadingColor();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
//                    robot.intake.setIntakePower(-1);
//                    robot.intake.intakeWrist.dropToAutoMidPos();
//                    robot.outtake.lock();
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
//                    robot.intake.setIntakePower(0);
//                    robot.intake.retractIntakeWrist();
//                })
//                .splineToConstantHeading(new Vector2d(20, -12 * reflect), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, endPose.getY()), Math.toRadians(0))
//
//                // lift
//                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
//                    robot.outtake.setTargetPixelHeight(pixelHeight);
//                })
//                // wrist back
//                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
//                    robot.outtake.extendWrist();
//                })
//                // turn turret
//                .UNSTABLE_addTemporalMarkerOffset(TURN_TURRET_TIME, () -> {
//                    robot.outtake.setTurretAngle(270 + turretAngleDelta * reflect);
//                })
//
//                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
//                .splineToConstantHeading(endPose.vec(), Math.toRadians(0))
////                .addTemporalMarker(() -> robot.drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE))
//
//                // release pixel
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.outtake.unlockFrontLockBack();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
//                    robot.outtake.resetLock();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(TOTAL_DEPOSIT_TIME, () -> {
//                    robot.outtake.incrementTargetHeight(1);
//                })
//
//                .waitSeconds(TOTAL_DEPOSIT_TIME)
//                .build();
//    }
//}
