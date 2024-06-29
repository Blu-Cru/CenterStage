package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenterAfterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackPerimeter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackPerimeterAfterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterIntakeCenterStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositCenterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositPerimeterBackstage;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositPerimeterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.PerimeterIntakeCloseStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.PerimeterIntakeFailsafe;
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
    int depositCount;
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

        intakeFailsafePath = new PerimeterIntakeFailsafe().build();
        intakeCloseAfterFailedPath = new PerimeterIntakeCloseStack(0, 2, 25).build();

        parkPath = new PIDPathBuilder().addMappedPoint(42, 61, 180).build();
    }

    public PerimeterCycleBackdropConfig() {

        preloadPaths = new HashMap<>();

        depositCount = 0;

        stateMachine = new StateMachineBuilder()
                // PRELOADS

                .state(State.PLACING_PRELOADS)
                .onEnter(() -> {
                    logTransitionTo(State.PLACING_PRELOADS);
                })
                .loop(() -> {
                    robot.drivetrain.updateAprilTags();
                })
                .transition(() -> currentPath.isDone(), State.TO_STACK, () -> {
                    currentPath = preloadBackdropToStackPath.start();
                })

                // TO STACK STATE

                .state(State.TO_STACK)
                .onEnter(() -> {
                    logTransitionTo(State.TO_STACK);
                    Robot.getInstance().intake.startReadingColor();
                })
                .loop(() -> {
//                dt.updateAprilTags();
                })
                .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                    if(Globals.stackClosePixels > 1) currentPath = intakeCloseStackPath.start();
                    else currentPath = intakeCenterStackPath.start();
                })
//                .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE
//                        && dt.pose.getX() < 20 && dt.pose.getX() > -30, CenterCycleBackdropConfig.State.CRASH_TO_STACK_FAILSAFE, () -> {
//                    if(dt.pose.getX() > -8) currentPath = crashTrussBackdropFailsafePath.start();
//                    else currentPath = crashTrussMiddleFailsafeToIntakePath.start();
//
//                    if(Angle.normDelta(dt.pose.getHeading() - Math.PI) > 0) {
//                        dt.correctY(-CRASH_CORRECTION_Y);
//                        Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + -CRASH_CORRECTION_Y);
//                    } else {
//                        dt.correctY(CRASH_CORRECTION_Y);
//                        Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + CRASH_CORRECTION_Y);
//                    }
//                })

                // INTAKING STATE

                .state(State.INTAKING)
                .onEnter(() -> {
                    logTransitionTo(State.INTAKING);
                })
                .transition(() -> currentPath.isDone(), State.INTAKE_FAILSAFE, () -> {
                    currentPath = intakeFailsafePath.start();
                })
                .transition(() -> robot.intake.isFull(), State.TO_BACKDROP, () -> {
                    try{
                        CommandScheduler.getInstance().cancelAll();
                    } catch (Exception e){}
                    currentPath = stackToBackdropPath.start();
                    robot.outtake.lock();
                    Globals.stackFarPixels -= 2;
                })
                .transition(() -> runtime.seconds() > 27.0, State.TO_BACKDROP, () -> {
                    currentPath = new StackToBackdropPerimeter().build().start();
                })

                // TO BACKDROP STATE

                .state(State.TO_BACKDROP)
                .onEnter(() -> {
                    logTransitionTo(State.TO_BACKDROP);
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() < 27.7, State.DEPOSITING, () -> {
                    currentPath = new DepositPerimeterCycle(depositCount + 1, 4).build().start();
                })
                .transition(() -> currentPath.isDone() &&
                        (runtime.seconds() > 27.7 && runtime.seconds() < 29.3) &&
                        !robot.intake.isEmpty(), State.DEPOSITING_BACKSTAGE, () -> {
                    currentPath = depositBackstagePath.start();
                })
                .transition(() -> currentPath.isDone() &&
                        (runtime.seconds() > 29.3 || (robot.intake.isEmpty() && runtime.seconds() > 27.7)),
                        State.PARKING, () -> {
                    currentPath = parkPath.start();
                })
//                .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE
//                        && dt.pose.getX() < 20 && dt.pose.getX() > -30, CenterCycleBackdropConfig.State.CRASH_TO_BACKDROP_FAILSAFE, () -> {
//                    if(dt.pose.getX() > -16) currentPath = crashTrussMiddleFailsafeToBackdropPath.start();
//                    else currentPath = crashTrussStackFailsafePath.start();
//
//                    // CORRECT FOR CRASH
//                    if(Angle.normDelta(dt.pose.getHeading() - Math.PI) > 0) {
//                        dt.correctY(CRASH_CORRECTION_Y);
//                        Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + CRASH_CORRECTION_Y);
//                    } else {
//                        dt.correctY(-CRASH_CORRECTION_Y);
//                        Log.i("CenterCycleBackdropConfig", "Correcting Y by: " + -CRASH_CORRECTION_Y);
//
//                    }
//                })

                // DEPOSITING STATE

                .state(State.DEPOSITING)
                .onEnter(() -> {
                    logTransitionTo(State.DEPOSITING);
                    robot.intake.stopReadingColor();
                })
                .loop(() -> {
                    robot.drivetrain.updateAprilTags();
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() < 25, State.TO_STACK, () -> {
                    new OuttakeRetractCommand(1.5).schedule();
                    depositCount++;
                    currentPath = backdropToStackPath.start();
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() >= 25, State.PARKING, () -> {
                    new OuttakeRetractCommand(1.5).schedule();
                    currentPath = parkPath.start();
                })

                // CRASH FAILSAFE STATE

//                .state(State.CRASH_TO_STACK_FAILSAFE)
//                .onEnter(() -> {
//                    logTransitionTo(State.CRASH_TO_STACK_FAILSAFE);
//                })
//                .transition(() -> currentPath.isDone() && dt.getAbsHeadingError() < TRUSS_HEADING_FAILSAFE_TOLERANCE, CenterCycleBackdropConfig.State.TO_STACK, () -> {
//                    currentPath = crashToStackRecoveryPath.start();
//                })

                .state(State.CRASH_TO_BACKDROP_FAILSAFE)
                .onEnter(() -> {
                    logTransitionTo(State.CRASH_TO_BACKDROP_FAILSAFE);
                })
//                .transition(() -> currentPath.isDone() && dt.getAbsHeadingError() < TRUSS_HEADING_FAILSAFE_TOLERANCE, CenterCycleBackdropConfig.State.TO_BACKDROP, () -> {
//                    currentPath = crashToBackdropRecoveryPath.start();
//                })

                .state(State.INTAKE_FAILSAFE)
                .onEnter(() -> {
                    logTransitionTo(State.INTAKE_FAILSAFE);
                })
                .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                    currentPath = intakeCloseAfterFailedPath.start();
                })
                .transition(() -> runtime.seconds()>27, State.TO_BACKDROP, () -> {
                    currentPath = new StackToBackdropPerimeter().build().start();
                })
                .transition(() -> Robot.getInstance().intake.isFull(), State.TO_BACKDROP, () -> {
                    try{
                        CommandScheduler.getInstance().cancelAll();
                    } catch (Exception e){}
                    currentPath = stackToBackdropPath.start();
                    Robot.getInstance().outtake.lock();
                    Globals.stackFarPixels -= 2;
                })

                .state(State.DEPOSITING_BACKSTAGE)
                .onEnter(() -> logTransitionTo(State.DEPOSITING_BACKSTAGE))
                .transition(() -> currentPath.isDone(), State.DONE)

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
    }

    public void run() {
        stateMachine.update();
        currentPath.run();
    }

    public void start(Randomization randomization) {
        robot = Robot.getInstance();
        currentPath = preloadPaths.get(randomization);
        stateMachine.start();
        stateMachine.setState(State.PLACING_PRELOADS);
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
