package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Autonomous(name = "Auto", group = "1")
public class Auto extends BCLinearOpMode {
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
                if(stickyG1.y) Globals.setAlliance(Globals.alliance.flip());
                if(stickyG1.b) Globals.side = Globals.side.flip();
                if(stickyG1.a) Globals.autoType = Globals.autoType.cycle();
                if(stickyG1.x) Globals.parkType = Globals.parkType.cycle();

                if(opModeIsActive()) requestOpModeStop();

                Globals.autoConfigTelemetry(telemetry);
            })
            .transition(() -> stickyG1.right_stick_button, State.DETECTION, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                telemetry.addLine("Building Paths . . .");
                telemetry.update();
                config = AutoConfig.config();
                config.build();
                telemetry.addData("Initializing CV . . .", "");
                telemetry.update();
                Globals.setAutoStartPose();
                if(Globals.side == Side.AUDIENCE) addPurplePixelHolder();
                addCVMaster();
                cvMaster.detectProp();
            })
            .state(State.DETECTION)
            .loop(() -> {
                propPosition = cvMaster.propDetector.position;
                telemetry.addLine("Left stick button to go back to config");
                cvMaster.propDetector.telemetry(telemetry);
                Globals.autoConfigStatus(telemetry);
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
                drivetrain.initializePose();
                Globals.startAuto();
                config.start(Globals.getRandomization(propPosition));
                cvMaster.detectTag();
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
        optimizeTelemetry();
        stateMachine.setState(State.CONFIG);
        stateMachine.start();
    }

    @Override
    public void onStart() {
        drivetrain.initializePose();
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
        Globals.startPose = drivetrain.pose;
    }

    @Override
    public void telemetry() {
        telemetry.addData("Auto state: ", stateMachine.getState());
        config.telemetry(telemetry);
    }
}
