package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.util.Utils;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropThroughTrussCenter;

import java.util.HashMap;

public class CenterCycleBackdropConfig extends AutoConfig {
    static final Pose2d START_POSE = Utils.mapPose(12, 61, Math.toRadians(90));
    static final double TRUSS_HEADING_FAILSAFE_TOLERANCE = Math.toRadians(40);

    Path farPreloadPath, centerPreloadPath, closePreloadPath;
    Path preloadPath;
    Path backdropToStackPath, stackToBackdropPath;
    Path backdropFailsafePath, stackFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath, crashTrussMiddleFailsafePath;
    Path currentFailsafePath;

    Drivetrain dt;

    HashMap<Randomization, Path> preloadPaths = new HashMap<Randomization, Path>() {{
        put(Randomization.FAR, farPreloadPath);
        put(Randomization.CENTER, centerPreloadPath);
        put(Randomization.CLOSE, closePreloadPath);
    }};

    enum State {
        PLACING_PRELOADS,
        TO_STACK,
        INTAKING,
        TO_BACKDROP,
        DEPOSITING,
        PARKING,
        DONE,

        // Failsafe states
        DEPOSIT_FAILSAFE,
        CRASH_FAILSAFE,
        INTAKE_FAILSAFE,
    }


    Path currentPath;

    StateMachine stateMachine = new StateMachineBuilder()
            // PRELOADS

            .state(State.PLACING_PRELOADS)
            .loop(() -> {
                preloadPath.run();
                dt.updateAprilTags();
            })
            .transition(() -> preloadPath.isDone(), State.TO_STACK, () -> {
                backdropToStackPath.start();
            })

            // BACKDROP DRIVING TO STACK

            .state(State.TO_STACK)
            .loop(() -> {
                backdropToStackPath.run();
                dt.updateAprilTags();
            })
            .transition(() -> backdropToStackPath.isDone(), State.INTAKING, () -> {
//                dt.intake.start();
            })
            .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE, State.CRASH_FAILSAFE)
            .build();

    public CenterCycleBackdropConfig() {

    }

    public void build() {
        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());

        backdropToStackPath = new BackdropThroughTrussCenter().build();

        crashTrussBackdropFailsafePath = new PIDPathBuilder().addMappedPoint(10, 12, 180).build();
        crashTrussStackFailsafePath = new PIDPathBuilder().addMappedPoint(-34, 12, 180).build();
        crashTrussMiddleFailsafePath = new PIDPathBuilder().addMappedPoint(-12, 12, 180).build();

        dt = Robot.getInstance().drivetrain;
    }

    public void run() {
        stateMachine.update();
    }

    public void start(Randomization randomization) {
        preloadPath = preloadPaths.get(randomization);
        stateMachine.setState(State.PLACING_PRELOADS);
        stateMachine.start();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Backdrop Center Cycle");
        telemetry.addData("State", stateMachine.getState());
    }
}
