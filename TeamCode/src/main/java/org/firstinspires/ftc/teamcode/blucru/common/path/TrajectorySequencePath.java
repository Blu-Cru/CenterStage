package org.firstinspires.ftc.teamcode.blucru.common.path;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectorySequencePath implements Path {
    TrajectorySequence sequence;
    public TrajectorySequencePath(TrajectorySequence sequence) {
        this.sequence = sequence;
    }

    @Override
    public Path start() {
        Robot.getInstance().drivetrain.followTrajectorySequenceAsync(this.sequence);
        Robot.getInstance().drivetrain.idle();

        return this;
    }

    @Override
    public void run() {
        Robot.getInstance().drivetrain.updateTrajectory();
    }

    @Override
    public boolean isDone() {
        return !Robot.getInstance().drivetrain.isBusy();
    }

    @Override
    public void breakPath() {
        Robot.getInstance().drivetrain.breakFollowing();
    }
}
