package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Config
@TeleOp(name = "Deposit position test", group = "test")
public class DepositPositionTest extends BCLinearOpMode {
    public static double depositX = 0;
    public static double depositY = 0;
    public static double depositHeading = 0;

    Pose2d depositPose = new Pose2d(0, 0, 0);

    private enum State {
        GETTING_ANGLE,
        SCANNING,
        LIFTING,
        DRIVING_TO_OUTTAKE,
        DONE_OUTTAKING
    }

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.GETTING_ANGLE)
            .loop(() -> {
                double vert = -gamepad1.left_stick_y;
                double horz = gamepad1.left_stick_x;
                double rot = -gamepad1.right_stick_x;

                drivetrain.teleOpDrive(horz, vert, rot);

                if(gamepad1.right_stick_button) {
                    drivetrain.resetHeading(Math.toRadians(90));
                    gamepad1.rumble(200);
                }
            })
            .transition(() -> stickyG1.a, State.SCANNING, () -> {
                gamepad1.rumble(200);
                cvMaster.detectTag();
            })
            .state(State.SCANNING)
            .loop(() -> {
                drivetrain.updateAprilTags(cvMaster.tagDetector);
                double vert = -gamepad1.left_stick_y;
                double horz = gamepad1.left_stick_x;
                double rot = -gamepad1.right_stick_x;

                drivetrain.teleOpDrive(horz, vert, rot);

                if(gamepad1.right_stick_button) {
                    drivetrain.resetHeading(Math.toRadians(90));
                    gamepad1.rumble(200);
                }
            })
            .transition(() -> stickyG1.a, State.LIFTING, () -> {
                gamepad1.rumble(200);

                CommandScheduler.getInstance().schedule(new OuttakeExtendCommand(2));
            })
            .state(State.LIFTING)
            .transitionTimed(1.0, State.DRIVING_TO_OUTTAKE, () -> gamepad1.rumble(200))
            .state(State.DRIVING_TO_OUTTAKE)
            .loop(() -> {
                drivetrain.pidTo(depositPose);
            })
            .transition(() -> stickyG1.a, State.DONE_OUTTAKING, () -> {
                gamepad1.rumble(200);
            })
            .state(State.DONE_OUTTAKING)
            .loop(() -> {
                if(stickyG1.dpad_down) {
                    CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
                }
            })
            .transition(() -> stickyG1.a, State.SCANNING, () -> gamepad1.rumble(200))
            .build();

    @Override
    public void initialize() {
        addDrivetrain(false);
        addOuttake();
        addIntakeWrist();
        addCVMaster();

        Poses.setAlliance(Alliance.BLUE);
        depositX = Poses.DEPOSIT_CENTER_POSE.getX();
        depositY = Poses.DEPOSIT_CENTER_POSE.getY();
        depositHeading = Poses.DEPOSIT_CENTER_POSE.getHeading();
        updateDepositPose();

        stateMachine.setState(State.GETTING_ANGLE);
        stateMachine.start();
    }

    @Override
    public void periodic() {
        drivetrain.ftcDashDrawCurrentPose();
        updateDepositPose();
        stateMachine.update();
    }

    public void updateDepositPose() {
        depositPose = new Pose2d(depositX, depositY, depositHeading);
    }

    @Override
    public void telemetry() {
        telemetry.addData("state: ", stateMachine.getState());
    }
}
