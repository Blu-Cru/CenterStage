package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadIntake;

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
    public CenterCycleAudienceConfig() {

    }

    public void build() {
        preloadIntakeFar = new AudienceFarPreloadIntake().build();
    }

    public void start(Randomization randomization) {
        preloadIntakeFar.start();
    }

    public void run() {
        preloadIntakeFar.run();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Audience Center Cycle");
//        telemetry.addData("State", stateMachine.getState());
    }
}
