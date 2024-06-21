package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterToBackdropPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceClosePreloadIntake;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterDepositFailsafe;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterIntakeFailsafe;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositCenterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.IntakeFarStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.StackToBackdropCenter;
import java.util.HashMap;

public class CenterCycleAudienceConfig extends AutoConfig {
    enum State{
        PRELOAD_INTAKING,
        TO_BACKDROP_PRELOAD,
        WAITING_FOR_PRELOAD_DEPOSIT,
        PRELOAD_DEPOSITING,
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

    HashMap<Randomization, Path> preloadIntakePaths;
    HashMap<Randomization, Path> preloadDepositPaths;
    Path backdropToStackPath, stackToBackdropPath,
            intakePath, depositPath, parkPath,

            toBackdropPreloadPath,

            depositFailsafePath, intakeFailsafePath,
            crashTrussBackdropFailsafePath, crashTrussStackFailsafePath,
            crashTrussMiddleFailsafeToIntakePath, crashTrussMiddleFailsafeToBackdropPath,
            crashToStackRecoveryPath, crashToBackdropRecoveryPath;

    Path currentPath;

    StateMachine stateMachine;

    public CenterCycleAudienceConfig() {
        preloadIntakePaths = new HashMap<>();
        preloadDepositPaths = new HashMap<>();
        stateMachine = new StateMachineBuilder()
                .state(State.PRELOAD_INTAKING)
                .onEnter(() -> logTransitionTo(State.PRELOAD_INTAKING))
                .transition(() -> currentPath.isDone(), State.TO_BACKDROP_PRELOAD, () -> currentPath = toBackdropPreloadPath.start())

                // TODO: create failsafe for if intake fails

                // TO BACKDROP PRELOAD
                .state(State.TO_BACKDROP_PRELOAD)
                .onEnter(() -> logTransitionTo(State.TO_BACKDROP_PRELOAD))
                .transition(() -> currentPath.isDone(), State.WAITING_FOR_PRELOAD_DEPOSIT)

                // WAITING FOR PRELOAD DEPOSIT
                .state(State.WAITING_FOR_PRELOAD_DEPOSIT)
                .onEnter(() -> logTransitionTo(State.WAITING_FOR_PRELOAD_DEPOSIT))

                // TODO: create preload deposit transition
                .transition(() -> Robot.getInstance().cvMaster.tagDetector.getDetections().size() > 2, State.PRELOAD_DEPOSITING)
                // TODO: create preload deposit path
                .transitionTimed(5, State.PRELOAD_DEPOSITING)
                .build();
    }

    public void build() {
        preloadIntakePaths.put(Randomization.CLOSE, new AudienceClosePreloadIntake().build());
        preloadIntakePaths.put(Randomization.CENTER, new AudienceCenterPreloadIntakeForCenter().build());
        preloadIntakePaths.put(Randomization.FAR, new AudienceFarPreloadIntakeForCenter().build());

        toBackdropPreloadPath = new AudienceCenterToBackdropPreload().build();

        backdropToStackPath = new BackdropToStackCenter().build();
        stackToBackdropPath = new StackToBackdropCenter().build();
        intakePath = new IntakeFarStack().build();
        depositPath = new DepositCenterCycle().build();
        parkPath = new PIDPathBuilder().addMappedPoint(42, 10, 220).build();

        intakeFailsafePath = new CenterIntakeFailsafe().build();
        depositFailsafePath = new CenterDepositFailsafe().build();
    }

    public void start(Randomization randomization) {
        currentPath = preloadIntakePaths.get(randomization);
    }

    public void run() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Audience Center Cycle");
//        telemetry.addData("State", stateMachine.getState());
    }

    public void logTransitionTo(Enum to) {
        Log.i("CenterCycleAudienceConfig", "State transitioning to " + to);
    }
}
