package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@TeleOp(name = "RR with AprilTag test", group = "test")
public class RRandATagTest extends KLinearOpMode {
    private enum State {
        RUNNING,
        IDLE,
        PID_TO_DEPOSIT
    }
    TrajectorySequence traj;
    PreloadDeposits preloadDeposits;

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.IDLE)
            .loop(() -> {
                drivetrain.idle();
                drivetrain.drivePIDClip(0,0,0);
            })
            .transition(() -> stickyG1.a, State.RUNNING, () -> {
                drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
                drivetrain.resetHeading(Math.toRadians(90));
                drivetrain.followTrajectorySequenceAsync(traj);
            })
            .state(State.RUNNING)
            .loop(() -> {
                // follow trajectory
                try {drivetrain.updateTrajectory();}
                // if the trajectory is interrupted, stop the op mode
                catch (Exception e) {requestOpModeStop();}
            })
            .transition(() -> !drivetrain.isBusy(), State.PID_TO_DEPOSIT, () -> {
                drivetrain.breakFollowing();
            })
            .state(State.PID_TO_DEPOSIT)
            .loop(() -> {
                drivetrain.ftcDashDrawCurrentPose();
                drivetrain.pidTo(traj.end());
            })
            .transition(() -> stickyG1.a, State.IDLE, () -> drivetrain.idle())
            .build();

    @Override
    public void initialize() {
        addDrivetrain(false);
        addCVMaster();

        Poses.setAlliance(Alliance.BLUE);
        preloadDeposits = new PreloadDeposits(-1);
        traj = preloadDeposits.rrTest(robot);
        cvMaster.detectTag();

        stateMachine.start();
        stateMachine.setState(State.IDLE);
    }

    @Override
    public void periodic() {
        stateMachine.update();
        drivetrain.updateAprilTags(cvMaster.tagDetector);
    }

    @Override
    public void telemetry() {
        telemetry.addData("state:", stateMachine.getState());
    }
}
