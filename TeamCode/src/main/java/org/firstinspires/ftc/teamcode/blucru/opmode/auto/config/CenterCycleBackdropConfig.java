package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.util.Utils;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;

import java.util.HashMap;

public class CenterCycleBackdropConfig extends AutoConfig {
    static final Pose2d START_POSE = Utils.mapPose(12, 61, Math.toRadians(90));

    Path farPreloadPath, centerPreloadPath, closePreloadPath;
    Path preloadPath;
    Path backdropToStackPath, stackToBackdropPath;
    Path backdropFailsafePath, stackFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath, crashTrussMiddleFailsafePath;

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
            .state(State.PLACING_PRELOADS)
            .loop(() -> {
                preloadPath.run();
            })
            .transition(() -> preloadPath.isDone(), State.DONE)
            .build();

    public CenterCycleBackdropConfig() {

    }

    public void build() {
        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());
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
