package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "PID Path test", group = "2")
public class PIDPathTest extends BCLinearOpMode {
    private enum State {
        RESETTING,
        FOLLOWING_TRAJECTORY,
        FOLLOWING_PID
    }

    PIDPath pidPath;

    double pidStartTime, pidTotalTime;

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.RESETTING)
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
            .state(State.FOLLOWING_PID)
            .transition(() -> stickyG1.a, State.RESETTING, ()-> {
                drivetrain.idle();
                pidPath.breakPath();
                drivetrain.driveScaled(0,0,0);
                intake.setIntakePower(0);
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
        addPurplePixelHolder();
        Globals.setAlliance(Alliance.BLUE);

        pidPath = new PIDPathBuilder()
                .setPower(0.45)
                .addMappedPoint(-55, 42, 135,6)
                .addMappedPoint(-58, 30, 180, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new PurplePixelRetractCommand(),
                        new DropdownPartialRetractCommand()
                ))
                .addMappedPoint(-53, 21, 220,2)

                .waitMillis(350)
                .schedule(new IntakeCommand(4, 1))
                .addMappedPoint(Field.INTAKE_X, 12, 180, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new DropdownCommand(3),
                        new WaitCommand(400),
                        new DropdownCommand(0)
                ))
                .waitMillis(1000)
                .build();

        stateMachine.setState(State.RESETTING);
        stateMachine.start();

        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        stateMachine.update();
        drivetrain.ftcDashDrawCurrentPose();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state:", stateMachine.getState());
        telemetry.addData("pid total time: ", pidTotalTime);
        pidPath.telemetry(telemetry);
    }
}
