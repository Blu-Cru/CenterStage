package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositCenterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.IntakeFarStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.StackToBackdropCenter;

import java.util.HashMap;

public class CenterCycleBackdropConfig extends AutoConfig {
    static final double TRUSS_HEADING_FAILSAFE_TOLERANCE = Math.toRadians(40);

    Path farPreloadPath, centerPreloadPath, closePreloadPath;
    Path backdropToStackPath, stackToBackdropPath;
    Path backdropFailsafePath, stackFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath, crashTrussMiddleFailsafePath;
    Path crashToStackRecoveryPath, crashToBackdropRecoveryPath;
    Path intakePath;
    Path depositPath;

    int closeStackPixels = 5;
    int centerStackPixels = 5;
    int farStackPixels = 5;

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
        CRASH_TO_STACK_FAILSAFE,
        INTAKE_FAILSAFE,
    }

    Path currentPath;

    StateMachine stateMachine = new StateMachineBuilder()
            // PRELOADS

            .state(State.PLACING_PRELOADS)
            .loop(() -> {
                dt.updateAprilTags();
            })
            .transition(() -> currentPath.isDone(), State.TO_STACK, () -> {
                currentPath = backdropToStackPath.start();
            })

            // DRIVING TO STACK

            .state(State.TO_STACK)
            .loop(() -> {
                dt.updateAprilTags();
            })
            // TODO: Transition to intake path
            .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                currentPath = intakePath.start();
            })
            //            .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE, State.CRASH_TO_STACK_FAILSAFE, () -> {
//                if(dt.pose.getX() > -8) currentPath = crashTrussBackdropFailsafePath.start();
//                else currentPath = crashTrussMiddleFailsafePath.start();
//            })

            // INTAKING STATE

            .state(State.INTAKING)
            .transition(() -> currentPath.isDone(), State.TO_BACKDROP, () -> {
                currentPath = stackToBackdropPath.start();
            })

            // TO BACKDROP STATE

            .state(State.TO_BACKDROP)
            .transition(() -> currentPath.isDone(), State.DEPOSITING, () -> {
                currentPath = depositPath.start();
            })

            // DEPOSITING STATE

            .state(State.DEPOSITING)
            .transition(() -> currentPath.isDone(), State.DONE, () -> {
                new OuttakeRetractCommand().schedule();
            })


            // CRASH FAILSAFE STATE

//            .state(State.CRASH_TO_STACK_FAILSAFE)
////            .transition(currentPath::isDone, State.TO_STACK, () -> {
////                currentPath = crashToStackRecoveryPath.start();
////            })

            .state(State.DONE)
            .build();

    public CenterCycleBackdropConfig() {

    }

    public void build() {
        dt = Robot.getInstance().drivetrain;

        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());

        backdropToStackPath = new BackdropToStackCenter().build();
        stackToBackdropPath = new StackToBackdropCenter().build();
        intakePath = new IntakeFarStack(4).build();
        depositPath = new DepositCenterCycle(2, 0).build();

        crashTrussBackdropFailsafePath = new PIDPathBuilder().addMappedPoint(10, 12, 180).build();
        crashTrussStackFailsafePath = new PIDPathBuilder().addMappedPoint(-34, 12, 180).build();
        crashTrussMiddleFailsafePath = new PIDPathBuilder().addMappedPoint(-12, 12, 180).build();

        crashToStackRecoveryPath = new PIDPathBuilder().addMappedPoint(-34, 12, 180).build();
        crashToBackdropRecoveryPath = new PIDPathBuilder().addMappedPoint(10, 12, 180).build();
    }

    public void run() {
        stateMachine.update();
        currentPath.run();
    }

    public void start(Randomization randomization) {
        currentPath = preloadPaths.get(randomization);
        stateMachine.start();
        stateMachine.setState(State.PLACING_PRELOADS);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Backdrop Center Cycle");
        telemetry.addData("State", stateMachine.getState());
    }
}
