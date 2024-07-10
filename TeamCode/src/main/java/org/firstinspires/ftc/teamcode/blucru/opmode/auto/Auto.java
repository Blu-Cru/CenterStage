package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;

@Config
@Autonomous(name = "Auto", group = "1")
public class Auto extends KLinearOpMode {
    public static double DELAY_SECS = 0;

    private enum State {
        CONFIG,
        BUILD,
        DETECTION,
        RUNNING,
        PAUSING,
        END
    }
    AutoConfig config;
    int propPosition;
    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.CONFIG)
            .loop(() -> {
                if(stickyG1.y || stickyG2.y) Globals.setAlliance(Globals.alliance.flip());
                if(stickyG1.b || stickyG2.b) Globals.side = Globals.side.flip();
                if(stickyG1.a || stickyG2.a) Globals.autoType = Globals.autoType.cycle();
                if(stickyG1.x || stickyG2.x) Globals.parkType = Globals.parkType.cycle();

                if(opModeIsActive()) requestOpModeStop();

                Globals.autoConfigTelemetry(telemetry);
            })
            .transition(() -> stickyG1.right_stick_button || stickyG2.right_stick_button, State.DETECTION, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                telemetry.update();
                telemetry.addLine("Building Paths . . .");
                telemetry.update();
                config = AutoConfig.config();
                config.build();
                telemetry.update();
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
                telemetry.addLine("Left stick button: back to config");
                cvMaster.propDetector.telemetry(telemetry);
                Globals.autoConfigStatus(telemetry);
            })
            .transition(() -> stickyG1.left_stick_button || stickyG2.left_stick_button, State.CONFIG, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                cvMaster.stop();
                cvMaster = null;
            })
            .transition(this::opModeIsActive, State.PAUSING, () -> {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                drivetrain.initializePose();
                Globals.startAuto();
                cvMaster.detectTag();
                FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.visionPortal, 30);
            })
            .state(State.PAUSING)
            .transitionTimed(DELAY_SECS, State.RUNNING, () -> {
                config.start(Globals.getRandomization(propPosition));
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
        optimizeTelemetry();
        stateMachine.setState(State.CONFIG);
        stateMachine.start();
        enableFTCDashboard();
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
        if(Globals.alliance == Alliance.RED) Globals.startPose = drivetrain.pose;
        else Globals.startPose = new Pose2d(drivetrain.pose.vec(), Angle.norm(drivetrain.pose.getHeading() + Math.PI));
        Log.i("Auto", "start pose set to" + Globals.startPose);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Auto state: ", stateMachine.getState());
        if(stateMachine.getState() == State.RUNNING) Globals.autoRunningTelemetry(telemetry);
        config.telemetry(telemetry);
    }
}
