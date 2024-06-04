package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

public class MTIAuto extends BCLinearOpMode {
    private enum State {
        CONFIG,
        BUILD,
        DETECTION,
        RUNNING,
        END
    }
    AutoConfig config;
    int propPosition;
    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.CONFIG)
            .loop(() -> {
                if(stickyG1.x) Globals.alliance = Globals.alliance.flip();
                if(stickyG1.y) Globals.side = Globals.side.flip();
                if(stickyG1.a) Globals.autoType = Globals.autoType.cycle();
                if(stickyG1.b) Globals.parkType = Globals.parkType.cycle();

                Globals.autoConfigTelemetry(telemetry);
            })
            .transition(() -> stickyG1.right_stick_button, State.DETECTION, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                telemetry.addLine("Building Paths . . .");
                config = AutoConfig.config();
                addCVMaster();
                cvMaster.detectProp();
            })
            .state(State.DETECTION)
            .loop(() -> {
                propPosition = cvMaster.propDetector.position;
                cvMaster.propDetector.telemetry(telemetry);
            })
            .transition(() -> stickyG1.left_stick_button, State.CONFIG, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                cvMaster.stop();
                cvMaster = null;
            })
            .transition(this::opModeIsActive, State.RUNNING, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                config.start();
                Globals.startTimer();
            })
            .state(State.RUNNING)
            .loop(() -> {
                config.run();
            })
            .build();

    @Override
    public void initialize() {
        addDrivetrain(false);
        addIntake();
        addOuttake();
        addPurplePixelHolder();
        stateMachine.setState(State.CONFIG);
        stateMachine.start();
    }

    @Override
    public void initLoop() {
        stateMachine.update();
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }

    @Override
    public void end() {
        Globals.setStartPose(drivetrain.getPoseEstimate());
    }
}
