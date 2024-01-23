package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class PlacementBackdropClose {
    public static TrajectorySequence build(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setVelConstraint(Constraints.FAST_VELOCITY)
                .lineTo(Poses.BACKDROP_PLACEMENT_CLOSE_POSE.vec())
                // release purple pixel

                .waitSeconds(0.5)
                .build();
    }
}
