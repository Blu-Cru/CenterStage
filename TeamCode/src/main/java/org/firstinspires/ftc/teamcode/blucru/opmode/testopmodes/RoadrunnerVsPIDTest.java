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
            .build();


    @Override
    public void initialize() {
        addDrivetrain(false);
        Poses.setAlliance(Alliance.BLUE);
        drivetrain.drivePower = 0.9;


    }
}
