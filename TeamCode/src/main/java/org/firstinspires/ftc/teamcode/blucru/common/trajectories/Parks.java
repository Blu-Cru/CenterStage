package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Parks {
    public static double reflect = 1;

    public static double TURRET_RETRACT_DELAY = 0.1;
    public static double WRIST_RETRACT_DELAY = 0.6;
    public static double LIFT_RETRACT_DELAY = 0.8;

    public Parks(Alliance alliance) {
        reflect = alliance == Alliance.BLUE ? -1 : 1;
    }

    public TrajectorySequence parkCloseBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
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

    public TrajectorySequence parkCloseBackdropCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
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

    public TrajectorySequence parkCloseBackdropFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
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

    public TrajectorySequence parkFarBackdropFar(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_FAR_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
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

    public TrajectorySequence parkFarBackdropCenter(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CENTER_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
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

    public TrajectorySequence parkFarBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.DEPOSIT_CLOSE_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
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
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(135 * reflect))
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
