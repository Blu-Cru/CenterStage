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
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload.BackdropFarPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.transition.BackdropToStackCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload.BackdropToStackCenterAfterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.deposit.CenterDepositFailsafe;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.intake.CenterIntakeCenterStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.intake.CenterIntakeFailsafe;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.deposit.DepositCenterBackstage;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.deposit.DepositCenterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.intake.CenterIntakeFarStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.transition.StackToBackdropCenter;

import java.util.HashMap;

public class CenterCycleBackdropConfig extends AutoConfig {
    static final double TRUSS_HEADING_FAILSAFE_TOLERANCE = Math.toRadians(15);
    static final double CRASH_CORRECTION_Y = 2;

    HashMap<Randomization, Path> preloadPaths;
    Path preloadBackdropToStackPath;
    Path backdropToStackPath, stackToBackdropPath;
    Path depositFailsafePath, intakeFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath,
            crashTrussMiddleFailsafeToIntakePath, crashTrussMiddleFailsafeToBackdropPath;
    Path crashToStackRecoveryPath, crashToBackdropRecoveryPath;
    Path intakeFarStackPath, intakeCenterStackPath, intakeFarAfterFailedPath;
    Path depositPath, depositBackstagePath;
    Path parkPath;

    Drivetrain dt;
    ElapsedTime runtime;

    int depositCount,
            intakeFailThisCycleCount, depositFailThisCycleCount;

    private enum State {
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

    Path currentPath;
    StateMachine stateMachine;

    public CenterCycleBackdropConfig() {
        dt = Robot.getInstance().drivetrain;
        preloadPaths = new HashMap<>();

        depositCount = 0;
        intakeFailThisCycleCount = 0;
        depositFailThisCycleCount = 0;
        runtime = Globals.runtime;

        stateMachine = new StateMachineBuilder()
                // PRELOADS

                .state(State.PLACING_PRELOADS)
                .onEnter(() -> {
                    logTransitionTo(State.PLACING_PRELOADS);
                })
                .loop(() -> {
                    dt.updateAprilTags();
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
                    if(Globals.stackFarPixels > 1) currentPath = intakeFarStackPath.start();
                    else currentPath = intakeCenterStackPath.start();
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
                .transition(() -> currentPath.isDone(), State.INTAKE_FAILSAFE, () -> {
                    currentPath = intakeFailsafePath.start();
                })
                .transition(() -> Robot.getInstance().intake.isFull(), State.TO_BACKDROP, () -> {
                    try{
                        CommandScheduler.getInstance().cancelAll();
                    } catch (Exception e){}
                    currentPath = stackToBackdropPath.start();
                    Robot.getInstance().outtake.lock();
                    Globals.stackFarPixels -= 2;
                })
                .transition(() -> runtime.seconds() > 27.0, State.TO_BACKDROP, () -> {
                    currentPath = new StackToBackdropCenter(true).build().start();
                })

                // TO BACKDROP STATE

                .state(State.TO_BACKDROP)
                .onEnter(() -> {
                    logTransitionTo(State.TO_BACKDROP);
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() < 27.7, State.DEPOSITING, () -> {
                    currentPath = new DepositCenterCycle(depositCount + 1, -4).build().start();
                })
                .transition(() -> currentPath.isDone() &&
                        (runtime.seconds() > 27.7 && runtime.seconds() < 29.3) &&
                        !Robot.getInstance().intake.isEmpty(), State.DEPOSITING_BACKSTAGE, () -> {
                    currentPath = depositBackstagePath.start();
                })
                .transition(() -> currentPath.isDone() && (runtime.seconds() > 29.3 || (Robot.getInstance().intake.isEmpty() && runtime.seconds() > 27.7)), State.PARKING, () -> {
                    currentPath = parkPath.start();
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
                    Robot.getInstance().intake.stopReadingColor();
                })
                .loop(() -> {
                    dt.updateAprilTags();
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

                .state(State.INTAKE_FAILSAFE)
                .onEnter(() -> {
                    logTransitionTo(State.INTAKE_FAILSAFE);
                })
                .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                    if(Globals.stackFarPixels > 1) currentPath = intakeFarAfterFailedPath.start();
                    else currentPath = new CenterIntakeCenterStack(0.5).build().start();
                })
                .transition(() -> runtime.seconds()>27, State.TO_BACKDROP, () -> {
                    currentPath = new StackToBackdropCenter(true).build().start();
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

    public void build() {
        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());

        preloadBackdropToStackPath = new BackdropToStackCenterAfterPreload().build();

        backdropToStackPath = new BackdropToStackCenter().build();
        stackToBackdropPath = new StackToBackdropCenter().build();
        intakeFarStackPath = new CenterIntakeFarStack().build();
        intakeCenterStackPath = new CenterIntakeCenterStack().build();

        intakeFarAfterFailedPath = new CenterIntakeFarStack(0,2,30).build();
        depositPath = new DepositCenterCycle().build();
        depositBackstagePath = new DepositCenterBackstage().build();
        parkPath = new PIDPathBuilder().addMappedPoint(43, 14, 200).build();

        intakeFailsafePath = new CenterIntakeFailsafe().build();
        depositFailsafePath = new CenterDepositFailsafe().build();

        crashTrussBackdropFailsafePath = new PIDPathBuilder().addMappedPoint(14, 12, 180).build();
        crashTrussStackFailsafePath = new PIDPathBuilder().addMappedPoint(-38, 12, 180).build();
        crashTrussMiddleFailsafeToIntakePath = new PIDPathBuilder().addMappedPoint(-12, 12, 210)
                .addMappedPoint(-8, 12, 180).build();
        crashTrussMiddleFailsafeToBackdropPath = new PIDPathBuilder().addMappedPoint(-12, 12, 150)
                .addMappedPoint(-16, 12, 180).build();

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
        telemetry.addData("runtime", runtime.seconds());
    }

    public void logTransitionTo(Enum to) {
        Log.i("CenterCycleBackdropConfig", "Transitioning to " + to);
    }
}
