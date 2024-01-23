package org.firstinspires.ftc.teamcode.blucru.common.trajectories;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Parks {
    public static double reflect = 1;

    public Parks(Alliance alliance) {
        reflect = alliance == Alliance.BLUE ? -1 : 1;
    }

    public TrajectorySequence parkCloseBackdropClose(Robot robot) {
        return robot.drivetrain.trajectorySequenceBuilder(Poses.BACKDROP_STARTING_POSE)
                .setConstraints(Constraints.FAST_VELOCITY, Constraints.FAST_ACCELERATION)
                .setTangent(Math.toRadians(90 * reflect))
                // retract lift

                .build();
    }
}
