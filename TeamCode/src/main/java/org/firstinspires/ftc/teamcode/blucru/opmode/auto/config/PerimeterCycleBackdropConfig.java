package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenterAfterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackPerimeter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackPerimeterAfterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositPerimeterBackstage;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositPerimeterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.PerimeterIntakeCloseStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.StackToBackdropCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.StackToBackdropPerimeter;

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

        intakeFailsafePath, intakeCloseAfterFailedPath,
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

        preloadBackdropToStackPath = new BackdropToStackPerimeterAfterPreload().build();
        backdropToStackPath = new BackdropToStackPerimeter().build();
        stackToBackdropPath = new StackToBackdropPerimeter().build();
        intakeCloseStackPath = new PerimeterIntakeCloseStack().build();
        depositPath = new DepositPerimeterCycle().build();
        depositBackstagePath = new DepositPerimeterBackstage().build();

        intakeCloseAfterFailedPath = new PerimeterIntakeCloseStack(0, 2, 25).build();

        parkPath = new PIDPathBuilder().addMappedPoint(42, 62, 180).build();


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
