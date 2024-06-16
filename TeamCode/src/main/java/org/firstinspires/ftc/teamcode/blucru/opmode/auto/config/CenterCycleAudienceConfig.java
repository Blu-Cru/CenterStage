package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceClosePreloadIntake;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadIntakeForCenter;

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

    HashMap<Randomization, Path> preloadPaths;
    Path currentPath;
    public CenterCycleAudienceConfig() {
        preloadPaths = new HashMap<>();
    }

    public void build() {
        preloadPaths.put(Randomization.CLOSE, new AudienceClosePreloadIntake().build());
        preloadPaths.put(Randomization.CENTER, new AudienceCenterPreloadIntakeForCenter().build());
        preloadPaths.put(Randomization.FAR, new AudienceFarPreloadIntakeForCenter().build());
    }

    public void start(Randomization randomization) {
    }

    public void run() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Audience Center Cycle");
//        telemetry.addData("State", stateMachine.getState());
    }
}
