package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Lift;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DepositTrajectories {
    public static double
            LIFT_TIME = -1.0,
            WRIST_EXTEND_TIME = -0.75,
            TURN_TURRET_TIME = -0.4,
            TOTAL_DEPOSIT_TIME = 0.3;
    double reflect;

    public DepositTrajectories (double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence depositCenterFromFarStack(Robot robot, double pixelHeight, double turretAngle) {
        return robot.drivetrain.trajectorySequenceBuilder(new Pose2d(Poses.STACK_SETUP_X, -12 * reflect, Math.toRadians(180 * reflect)))
                .setTangent(0)
                .setConstraints(Constraints.FAST_VEL, Constraints.FAST_ACCEL)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.intake.setIntakePower(-1);
                    robot.intake.intakeWrist.dropToAutoMidPos();
                    robot.outtake.lock();
                    robot.intake.stopReadingColor();
                })

                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.retractIntakeWrist();
                })
                .splineToConstantHeading(new Vector2d(30, -12 * reflect), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(Poses.BACKDROP_SETUP_X, Poses.DEPOSIT_FAR_Y * reflect), Math.toRadians(0))

                // lift
                .UNSTABLE_addTemporalMarkerOffset(LIFT_TIME, () -> {
                    robot.outtake.setTargetPixelHeight(pixelHeight);
                })
                // wrist back
                .UNSTABLE_addTemporalMarkerOffset(WRIST_EXTEND_TIME, () -> {
                    robot.outtake.extendWrist();
                })
                // turn turret
                .UNSTABLE_addTemporalMarkerOffset(TURN_TURRET_TIME, () -> {
                    robot.outtake.setTurretAngle(turretAngle);
                })

                .setConstraints(Constraints.SLOW_VEL, Constraints.SLOW_ACCEL)
                .splineToConstantHeading(Poses.DEPOSIT_FAR_POSE.vec(), Math.toRadians(0))
                .addTemporalMarker(() -> robot.drivetrain.lockTo(Poses.DEPOSIT_FAR_POSE))

                // release pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.unlock();
                })

                .waitSeconds(TOTAL_DEPOSIT_TIME)
                .build();
    }
}
