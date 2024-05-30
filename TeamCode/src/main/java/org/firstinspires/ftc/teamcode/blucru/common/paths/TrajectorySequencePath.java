package org.firstinspires.ftc.teamcode.blucru.common.paths;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.List;

public class TrajectorySequencePath extends TrajectorySequence implements Path {
    public TrajectorySequencePath(List<SequenceSegment> sequenceList) {
        super(sequenceList);
    }

    @Override
    public void init() {
        Robot.getInstance().drivetrain.followTrajectorySequenceAsync(this);
        Robot.getInstance().drivetrain.idle();
    }

    @Override
    public void follow() {
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
