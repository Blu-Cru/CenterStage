package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "RR vs PID test", group = "2")
public class RoadrunnerVsPIDTest extends BCLinearOpMode {
    private enum State {
        RESETTING,
        FOLLOWING_TRAJECTORY,
        FOLLOWING_PID
    }

    PIDPath pidPath;

    PreloadDeposits preloadDeposits;
    TrajectorySequence traj;

    double rrStartTime, rrTotalTime,
            pidStartTime, pidTotalTime;

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.RESETTING)
//            .transition(() -> stickyG1.a, State.FOLLOWING_TRAJECTORY, () -> {
//                drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
//                drivetrain.resetHeading(Poses.BACKDROP_STARTING_POSE.getHeading());
//                drivetrain.followTrajectorySequenceAsync(traj);
//                rrStartTime = System.currentTimeMillis();
//            })
            .transition(() -> stickyG1.b, State.FOLLOWING_PID, () -> {
                drivetrain.setPoseEstimate(Poses.AUDIENCE_STARTING_POSE);
                drivetrain.resetHeading(Poses.AUDIENCE_STARTING_POSE.getHeading());
                pidPath.start();
                pidStartTime = System.currentTimeMillis();
            })
            .loop(() -> {
                drivetrain.teleOpDrive(gamepad1);

                intake.intakeWrist.targetAngleDeg = 90;
            })
//            .state(State.FOLLOWING_TRAJECTORY)
//            .transition(() -> !drivetrain.isBusy(), State.RESETTING, () -> rrTotalTime = System.currentTimeMillis() - rrStartTime)
//            .transition(() -> stickyG1.a, State.RESETTING, () -> {
//                drivetrain.breakFollowing();
//                drivetrain.driveScaled(0,0,0);
//            })
//            .loop(() -> {
//                // follow trajectory
//                try {drivetrain.updateTrajectory();}
//                // if the trajectory is interrupted, stop the op mode
//                catch (Exception e) {requestOpModeStop();}
//            })
            .state(State.FOLLOWING_PID)
//            .transition(() -> pidPath.isDone(), State.RESETTING, () -> pidTotalTime = System.currentTimeMillis() - pidStartTime)
            .transition(() -> stickyG1.a, State.RESETTING, ()-> {
                drivetrain.idle();
                pidPath.breakPath();
                drivetrain.driveScaled(0,0,0);
                CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
            })
            .loop(() -> {
                drivetrain.ftcDashDrawCurrentPose();

                try {
                    pidPath.run();
                } catch (Exception e) {}

//                drivetrain.updateAprilTags(cvMaster.tagDetector);
            })
            .build();


    @Override
    public void initialize() {
        addDrivetrain(false);
        addIntake();
        addOuttake();
        addCVMaster();
        Globals.setAlliance(Alliance.RED);
        drivetrain.drivePower = 1;

        pidPath = new PIDPathBuilder()
                .setPower(0.8)
                .addMappedPoint(-38, 45, 120,6)
                .addMappedPoint(-33, 36, 160, false)
                .waitMillis(200)
                .addMappedPoint(-44, 30, 180, 6)
                .addMappedPoint(-58, 24, 180)
                .waitMillis(1000)
                .addMappedPoint(-45, 24,180)
                .build();

//        traj = preloadDeposits.rrTest(robot);

        stateMachine.setState(State.RESETTING);
        stateMachine.start();

        cvMaster.detectTag();
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
        telemetry.addData("dt", " is at target pose: " + drivetrain.isAtTargetPose());
    }
}
