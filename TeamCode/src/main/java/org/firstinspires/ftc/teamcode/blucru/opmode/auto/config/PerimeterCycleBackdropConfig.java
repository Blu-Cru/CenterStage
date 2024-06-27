package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;

import java.util.HashMap;

public class PerimeterCycleBackdropConfig extends AutoConfig {
    enum State {
        PLACING_PRELOADS,
        TO_STACK,
        INTAKING,
        TO_BACKDROP,
        DEPOSITING,
        DEPOSITING_BACKSTAGE,
        PARKING,
        DONE,

        // Failsafe states
        DEPOSIT_FAILSAFE,
        CRASH_TO_STACK_FAILSAFE,
        CRASH_TO_BACKDROP_FAILSAFE,
        INTAKE_FAILSAFE
    }

    HashMap<Randomization, Path> preloadPaths;

    Path preloadBackdropToStackPath,
        backdropToStackPath, stackToBackdropPath,
        intakeCloseStackPath, intakeCenterStackPath,
        depositPath, depositBackstagePath, parkPath,

        intakeFailsafePath, intakeAfterFailedPath,
        crashTrussBackdropFailsafePath, crashTrussStackFailsafePath,
        crashTrussMiddleToStackPath, crashTrussMiddleToBackdropPath;
    Path currentPath;
    StateMachine stateMachine;
    ElapsedTime runtime;
    Robot robot;

    public void build() {
        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());
    }

    public PerimeterCycleBackdropConfig() {

    }

    public void run() {
        stateMachine.update();
        currentPath.run();
    }

    public void start(Randomization randomization) {
        robot = Robot.getInstance();
        stateMachine.start();
//        stateMachine.setState(starting state)
        runtime = Globals.runtime;
    }

    public void telemetry(Telemetry tele) {
        tele.addLine("Audience Backdrop Cycle");
        tele.addData("State", stateMachine.getState());
        tele.addData("Runtime", runtime.seconds());
    }

    public void logTransitionTo(Enum to) {
        Log.i("PerimeterCycleBackdropConfig", "Transitioning to " + to);
    }
}
