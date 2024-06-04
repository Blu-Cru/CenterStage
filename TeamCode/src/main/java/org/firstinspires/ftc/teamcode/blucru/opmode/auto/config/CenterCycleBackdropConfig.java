package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class CenterCycleBackdropConfig extends AutoConfig {
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

    Path audiencePreloadPath, centerPreloadPath, backdropPreloadPath;
    Path backdropToStackPath, stackToBackdropPath;
    Path backdropFailsafePath, stackFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath, crashTrussMiddleFailsafePath;

    Path currentPath;

    StateMachine stateMachine = new StateMachineBuilder()
            .state(State.PLACING_PRELOADS)
            .build();

    public CenterCycleBackdropConfig() {

    }

    public void build() {

    }

    public void run() {

    }

    public void start() {
        stateMachine.setState(State.PLACING_PRELOADS);
        stateMachine.start();
    }
}
