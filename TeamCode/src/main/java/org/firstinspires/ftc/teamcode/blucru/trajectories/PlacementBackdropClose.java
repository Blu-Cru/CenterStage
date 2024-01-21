package org.firstinspires.ftc.teamcode.blucru.trajectories;

import org.firstinspires.ftc.teamcode.blucru.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class PlacementBackdropClose {
    public static TrajectorySequence build(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .splineTo(Poses.BACKDROP_PLACEMENT_FAR_POSE.vec(), Poses.BACKDROP_PLACEMENT_FAR_POSE.getHeading())
                .splineTo(Poses.BACKDROP_PLACEMENT_CLOSE_POSE.vec(), Poses.BACKDROP_PLACEMENT_CLOSE_POSE.getHeading())
                .splineTo(Poses.BACKDROP_PLACEMENT_CENTER_POSE.vec(), Poses.BACKDROP_PLACEMENT_CENTER_POSE.getHeading())
                .build();
    }
}
