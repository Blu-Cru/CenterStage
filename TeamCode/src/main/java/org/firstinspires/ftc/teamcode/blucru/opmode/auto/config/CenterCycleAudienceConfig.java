package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceClosePreloadIntake;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenter;
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

    HashMap<Randomization, Path> preloadPaths;
    Path backdropToStackPath, stackToBackdropPath,
            intakePath, depositPath, parkPath,

            depositFailsafePath, intakeFailsafePath,
            crashTrussBackdropFailsafePath, crashTrussStackFailsafePath,
            crashTrussMiddleFailsafeToIntakePath, crashTrussMiddleFailsafeToBackdropPath,
            crashToStackRecoveryPath, crashToBackdropRecoveryPath;

    Path currentPath;

    StateMachine stateMachine;

    public CenterCycleAudienceConfig() {
        preloadPaths = new HashMap<>();
        stateMachine = new StateMachineBuilder().build();
    }

    public void build() {
        preloadPaths.put(Randomization.CLOSE, new AudienceClosePreloadIntake().build());
        preloadPaths.put(Randomization.CENTER, new AudienceCenterPreloadIntakeForCenter().build());
        preloadPaths.put(Randomization.FAR, new AudienceFarPreloadIntakeForCenter().build());

        backdropToStackPath = new BackdropToStackCenter().build();
        stackToBackdropPath = new StackToBackdropCenter().build();
        intakePath = new IntakeFarStack(4).build();
        depositPath = new DepositCenterCycle().build();
        parkPath = new PIDPathBuilder().addMappedPoint(42, 10, 220).build();
    }

    public void start(Randomization randomization) {
        currentPath = preloadPaths.get(randomization);
    }

    public void run() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Audience Center Cycle");
//        telemetry.addData("State", stateMachine.getState());
    }
}
