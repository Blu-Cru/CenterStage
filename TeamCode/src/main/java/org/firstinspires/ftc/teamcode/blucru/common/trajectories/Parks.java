package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Parks {
    public static double reflect = 1;

    public static double TURRET_RETRACT_DELAY = 0.1;
    public static double WRIST_RETRACT_DELAY = 0.6;
    public static double LIFT_RETRACT_DELAY = 0.8;

    public Parks(double reflect) {
        this.reflect = reflect;
    }

    public TrajectorySequence parkClose(Robot robot, Pose2d initialPose) {
        return robot.drivetrain.trajectorySequenceBuilder(initialPose)
                .setConstraints(Constraints.NORMAL_VELOCITY, Constraints.NORMAL_ACCELERATION)
                .setTangent(Math.toRadians(225 * reflect))
//                // retract turret
//                .UNSTABLE_addTemporalMarkerOffset(TURRET_RETRACT_DELAY, () -> robot.outtake.centerTurret())
//                // retract wrist
//                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_DELAY, () -> robot.outtake.retractWrist())
//                // retract lift
//                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_DELAY, () -> robot.outtake.retractLift())
                .splineToConstantHeading(Poses.PARK_CLOSE_POSE.vec(), Math.toRadians(-90 * reflect))
                .build();
    }

    public TrajectorySequence parkCenter(Robot robot, Pose2d initialPose) {
        return robot.drivetrain.trajectorySequenceBuilder(initialPose)
                .setConstraints(Constraints.NORMAL_VELOCITY, Constraints.NORMAL_ACCELERATION)
                .setTangent(Math.toRadians(135 * reflect))
//                // retract turret
//                .UNSTABLE_addTemporalMarkerOffset(TURRET_RETRACT_DELAY, () -> robot.outtake.centerTurret())
//                // retract wrist
//                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_DELAY, () -> robot.outtake.retractWrist())
//                // retract lift
//                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_DELAY, () -> robot.outtake.retractLift())
                .splineToConstantHeading(Poses.PARK_FAR_POSE.vec(), Math.toRadians(90 * reflect))
                .build();
    }

    public TrajectorySequence retract(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
//                // retract turret
//                .UNSTABLE_addTemporalMarkerOffset(TURRET_RETRACT_DELAY, () -> robot.outtake.centerTurret())
//                // retract wrist
//                .UNSTABLE_addTemporalMarkerOffset(WRIST_RETRACT_DELAY, () -> robot.outtake.retractWrist())
//                // retract lift
//                .UNSTABLE_addTemporalMarkerOffset(LIFT_RETRACT_DELAY, () -> robot.outtake.retractLift())
                .waitSeconds(1.5)
                .build();
    }
}
