package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceClosePreloadIntake;

import java.util.HashMap;

public class CenterCycleAudienceConfig extends AutoConfig {
    enum State{
        PRELOAD_INTAKING,
        TO_BACKDROP_PRELOAD,
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
    Path preloadIntakeFar, preloadIntakeCenter, preloadIntakeClose;

    HashMap<Randomization, Path> preloadPaths;
    public CenterCycleAudienceConfig() {
        preloadPaths = new HashMap<>();
    }

    public void build() {
        preloadPaths.put(Randomization.CLOSE, new AudienceClosePreloadIntake().build());
        preloadPaths.put(Randomization.CENTER, new AudienceCenterPreloadIntakeForCenter().build());

        preloadIntakeClose = new AudienceClosePreloadIntake().build();
    }

    public void start(Randomization randomization) {
    }

    public void run() {
        preloadIntakeFar.run();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Audience Center Cycle");
//        telemetry.addData("State", stateMachine.getState());
    }
}
