package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class DepositFarFromBackdropClose {
    public static TrajectorySequence build(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_PLACEMENT_CLOSE_POSE)
                .setVelConstraint(Constraints.FAST_VELOCITY)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(30, -44 * Poses.reflect), 0)
                .splineToSplineHeading(Poses.DEPOSIT_FAR_POSE, 0)
                .build();
    }
}
