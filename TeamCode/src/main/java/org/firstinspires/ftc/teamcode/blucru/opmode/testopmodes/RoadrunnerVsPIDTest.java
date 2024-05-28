package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@TeleOp(name = "RR vs PID test", group = "test")
public class RoadrunnerVsPIDTest extends BCLinearOpMode {
    private enum State {
        RESETTING,
        FOLLOWING_TRAJECTORY,
        FOLLOWING_PID
    }

    ArrayList<Pose2d> pidPoints = new ArrayList<>();
    int pidIndex = 0;

    PreloadDeposits preloadDeposits;
    TrajectorySequence traj;

    double rrStartTime, rrTotalTime,
            pidStartTime, pidTotalTime;

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.RESETTING)
            .transition(() -> stickyG1.a, State.FOLLOWING_TRAJECTORY, () -> {
                drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
                drivetrain.resetHeading(Poses.BACKDROP_STARTING_POSE.getHeading());
                drivetrain.followTrajectorySequenceAsync(traj);
                rrStartTime = System.currentTimeMillis();
            })
            .transition(() -> stickyG1.b, State.FOLLOWING_PID, () -> {
                drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
                drivetrain.resetHeading(Poses.BACKDROP_STARTING_POSE.getHeading());
                drivetrain.pidTo(pidPoints.get(0));
                pidIndex = 0;
                pidStartTime = System.currentTimeMillis();
            })
            .loop(() -> {
                drivetrain.idle();
                drivetrain.driveScaled(0,0,0);
            })
            .state(State.FOLLOWING_TRAJECTORY)
            .transition(() -> !drivetrain.isBusy(), State.RESETTING, () -> rrTotalTime = System.currentTimeMillis() - rrStartTime)
            .transition(() -> stickyG1.a, State.RESETTING, () -> {
                drivetrain.breakFollowing();
                drivetrain.driveScaled(0,0,0);
            })
            .loop(() -> {
                // follow trajectory
                try {drivetrain.updateTrajectory();}
                // if the trajectory is interrupted, stop the op mode
                catch (Exception e) {requestOpModeStop();}
            })
            .state(State.FOLLOWING_PID)
            .transition(() -> pidIndex == pidPoints.size(), State.RESETTING, () -> pidTotalTime = System.currentTimeMillis() - pidStartTime)
            .transition(() -> stickyG1.a, State.RESETTING, ()-> {
                drivetrain.idle();
                drivetrain.driveScaled(0,0,0);
            })
            .loop(() -> {
                drivetrain.ftcDashDrawCurrentPose();

                try {
                    drivetrain.pidTo(pidPoints.get(pidIndex));
                } catch (Exception e) {}

                if(drivetrain.isAtTargetPose()) {
                    pidIndex++;
                }
            })
            .build();


    @Override
    public void initialize() {
        addDrivetrain(false);
        Poses.setAlliance(Alliance.BLUE);
        drivetrain.drivePower = 1;

        pidPoints.add(new Pose2d(12, -45 * Poses.reflect, Math.toRadians(-60 * Poses.reflect)));
        pidPoints.add(new Pose2d(8, -39 * Poses.reflect, Math.toRadians(-30 * Poses.reflect)));
//        pidPoints.add(new Pose2d(12, -41 * Poses.reflect, Math.toRadians(-45 * Poses.reflect)));
        pidPoints.add(new Pose2d(40, -30 * Poses.reflect, Math.toRadians(180)));

        preloadDeposits = new PreloadDeposits(-1);
        traj = preloadDeposits.rrTest(robot);

        stateMachine.setState(State.RESETTING);
        stateMachine.start();
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state:", stateMachine.getState());
        telemetry.addData("rr total time: ", rrTotalTime);
        telemetry.addData("pid total time: ", pidTotalTime);
        telemetry.addData("pid", "to point " + (pidIndex + 1) + " out of " + pidPoints.size());
        telemetry.addData("dt", " is at target pose: " + drivetrain.isAtTargetPose());
    }
}
