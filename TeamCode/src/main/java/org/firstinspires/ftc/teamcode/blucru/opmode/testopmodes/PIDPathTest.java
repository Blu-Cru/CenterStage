package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload.BackdropClosePreload;

@TeleOp(name = "PID Path test", group = "2")
public class PIDPathTest extends BluLinearOpMode {
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
                drivetrain.setPoseEstimate(Globals.startPose);
                drivetrain.resetHeading(Globals.startPose.getHeading());
                pidPath.start();
                pidStartTime = System.currentTimeMillis();
            })
            .loop(() -> {
                drivetrain.teleOpDrive(gamepad1);

//                intake.dropdown.targetAngleDeg = 90;
            })
            .state(State.FOLLOWING_PID)
            .transition(() -> stickyG1.a, State.RESETTING, ()-> {
                drivetrain.idle();
                pidPath.breakPath();
                drivetrain.driveScaled(0,0,0);
                intake.setPower(0);
                CommandScheduler.getInstance().schedule(new OuttakeRetractCommand());
            })
            .loop(() -> {
                drivetrain.ftcDashDrawCurrentPose();

                try {
                    pidPath.run();
                } catch (Exception e) {}

                drivetrain.updateAprilTags(cvMaster.tagDetector);
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
        Globals.side = Side.BACKDROP;

        pidPath = new BackdropClosePreload().build();

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
