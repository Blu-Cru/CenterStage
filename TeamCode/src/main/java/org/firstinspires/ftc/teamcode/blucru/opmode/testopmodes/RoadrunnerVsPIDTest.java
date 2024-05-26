package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class RoadrunnerVsPIDTest extends BCLinearOpMode {
    private enum State {
        RESETTING,
        FOLLOWING_TRAJECTORY,
        FOLLOWING_PID
    }

    ArrayList<Pose2d> pidPoints = new ArrayList<>();
    int pidIndex = 0;
    TrajectorySequence traj;

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.RESETTING)
            .transition(() -> stickyG1.a, State.FOLLOWING_TRAJECTORY, () -> {
                drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
                drivetrain.followTrajectorySequenceAsync(traj);
            })
            .transition(() -> stickyG1.b, State.FOLLOWING_PID, () -> {
                drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
            })
            .state(State.FOLLOWING_TRAJECTORY)
            .transition(() -> !drivetrain.isBusy(), State.RESETTING)
            .transition(() -> stickyG1.a, State.RESETTING, () -> drivetrain.breakFollowing())
            .loop(() -> {
                // follow trajectory
                try {drivetrain.updateTrajectory();}
                // if the trajectory is interrupted, stop the op mode
                catch (Exception e) {requestOpModeStop();}
            })
            .state(State.FOLLOWING_PID)
            .transition(() -> pidIndex == pidPoints.size(), State.RESETTING)
            .transition(() -> stickyG1.a, State.RESETTING)
            .build();


    @Override
    public void initialize() {
        addDrivetrain(false);
        Poses.setAlliance(Alliance.BLUE);
        drivetrain.drivePower = 0.9;


    }
}
