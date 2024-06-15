package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
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
    static final double TRUSS_HEADING_FAILSAFE_TOLERANCE = Math.toRadians(15);
    static final double CRASH_CORRECTION_Y = 2;

    Path backdropToStackPath, stackToBackdropPath;
    Path depositFailsafePath, intakeFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath,
            crashTrussMiddleFailsafeToIntakePath, crashTrussMiddleFailsafeToBackdropPath;
    Path crashToStackRecoveryPath, crashToBackdropRecoveryPath;
    Path intakePath;
    Path depositPath;
    Path parkPath;

    int closeStackPixels = 5;
    int centerStackPixels = 5;
    int farStackPixels = 5;

    Drivetrain dt;
    ElapsedTime runtime;

    HashMap<Randomization, Path> preloadPaths;

    private enum State {
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
        CRASH_TO_BACKDROP_FAILSAFE,
        INTAKE_FAILSAFE,
    }

    Path currentPath;

    StateMachine stateMachine = new StateMachineBuilder()
            // PRELOADS

            .state(State.PLACING_PRELOADS)
            .onEnter(() -> {
                logTransitionTo(State.PLACING_PRELOADS);
            })
            .loop(() -> {
                dt.updateAprilTags();
            })
            .transition(() -> currentPath.isDone(), State.TO_STACK, () -> {
                currentPath = backdropToStackPath.start();
            })

            // TO STACK STATE

            .state(State.TO_STACK)
            .onEnter(() -> {
                logTransitionTo(State.TO_STACK);
            })
            .loop(() -> {
//                dt.updateAprilTags();
            })
            .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                currentPath = intakePath.start();
            })
            .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE
                                && dt.pose.getX() < 20 && dt.pose.getX() > -30, State.CRASH_TO_STACK_FAILSAFE, () -> {
                if(dt.pose.getX() > -8) currentPath = crashTrussBackdropFailsafePath.start();
                else currentPath = crashTrussMiddleFailsafeToIntakePath.start();

                if(Angle.normDelta(dt.pose.getHeading() - Math.PI) > 0) {
                    dt.correctY(-CRASH_CORRECTION_Y);
                    Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + -CRASH_CORRECTION_Y);
                } else {
                    dt.correctY(CRASH_CORRECTION_Y);
                    Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + CRASH_CORRECTION_Y);
                }
            })

            // INTAKING STATE

            .state(State.INTAKING)
            .onEnter(() -> {
                logTransitionTo(State.INTAKING);
            })
            .transition(() -> currentPath.isDone(), State.TO_BACKDROP, () -> {
                currentPath = stackToBackdropPath.start();
            })

            // TO BACKDROP STATE

            .state(State.TO_BACKDROP)
            .onEnter(() -> {
                logTransitionTo(State.TO_BACKDROP);
            })
            .transition(() -> currentPath.isDone(), State.DEPOSITING, () -> {
                currentPath = depositPath.start();
            })
            .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE
                    && dt.pose.getX() < 20 && dt.pose.getX() > -30, State.CRASH_TO_BACKDROP_FAILSAFE, () -> {
                if(dt.pose.getX() > -16) currentPath = crashTrussMiddleFailsafeToBackdropPath.start();
                else currentPath = crashTrussStackFailsafePath.start();

                // CORRECT FOR CRASH
                if(Angle.normDelta(dt.pose.getHeading() - Math.PI) > 0) {
                    dt.correctY(CRASH_CORRECTION_Y);
                    Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + CRASH_CORRECTION_Y);
                } else {
                    dt.correctY(-CRASH_CORRECTION_Y);
                    Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + -CRASH_CORRECTION_Y);

                }
            })

            // DEPOSITING STATE

            .state(State.DEPOSITING)
            .onEnter(() -> {
                logTransitionTo(State.DEPOSITING);
            })
            .loop(() -> {
                dt.updateAprilTags();
            })
            .transition(() -> currentPath.isDone() && runtime.seconds() < 24, State.TO_STACK, () -> {
                new OuttakeRetractCommand().schedule();
                currentPath = backdropToStackPath.start();
            })
            .transition(() -> currentPath.isDone() && runtime.seconds() >= 24, State.PARKING, () -> {
                new OuttakeRetractCommand().schedule();
                currentPath = parkPath.start();
            })


            // CRASH FAILSAFE STATE

            .state(State.CRASH_TO_STACK_FAILSAFE)
            .onEnter(() -> {
                logTransitionTo(State.CRASH_TO_STACK_FAILSAFE);
            })
            .transition(() -> currentPath.isDone() && dt.getAbsHeadingError() < TRUSS_HEADING_FAILSAFE_TOLERANCE, State.TO_STACK, () -> {
                currentPath = crashToStackRecoveryPath.start();
            })

            .state(State.CRASH_TO_BACKDROP_FAILSAFE)
            .onEnter(() -> {
                logTransitionTo(State.CRASH_TO_BACKDROP_FAILSAFE);
            })
            .transition(() -> currentPath.isDone() && dt.getAbsHeadingError() < TRUSS_HEADING_FAILSAFE_TOLERANCE, State.TO_BACKDROP, () -> {
                currentPath = crashToBackdropRecoveryPath.start();
            })

            .state(State.PARKING)
            .onEnter(() -> {
                logTransitionTo(State.PARKING);
            })
            .transition(() -> currentPath.isDone(), State.DONE)

            .state(State.DONE)
            .onEnter(() -> {
                logTransitionTo(State.DONE);
            })
            .build();

    public CenterCycleBackdropConfig() {
        dt = Robot.getInstance().drivetrain;
        preloadPaths = new HashMap<>();
    }

    public void build() {
        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());

        backdropToStackPath = new BackdropToStackCenter().build();
        stackToBackdropPath = new StackToBackdropCenter().build();
        intakePath = new IntakeFarStack(4).build();
        depositPath = new DepositCenterCycle(2, 0).build();
        parkPath = new PIDPathBuilder().addMappedPoint(42, 10, 220).build();

        crashTrussBackdropFailsafePath = new PIDPathBuilder().addMappedPoint(6, 12, 180).build();
        crashTrussStackFailsafePath = new PIDPathBuilder().addMappedPoint(-38, 12, 180).build();
        crashTrussMiddleFailsafeToIntakePath = new PIDPathBuilder().addMappedPoint(-10, 12, 210)
                .addMappedPoint(-10, 12, 180).build();
        crashTrussMiddleFailsafeToBackdropPath = new PIDPathBuilder().addMappedPoint(-10, 12, 150)
                .addMappedPoint(-10, 12, 180).build();

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
        runtime = Globals.runtime;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Backdrop Center Cycle");
        telemetry.addData("State", stateMachine.getState());
        telemetry.addData("Runtime: ", runtime.seconds());
    }

    public void logTransitionTo(Enum to) {
        Log.i("CenterCycleBackdropConfig", "Transitioning to " + to);
    }
}
