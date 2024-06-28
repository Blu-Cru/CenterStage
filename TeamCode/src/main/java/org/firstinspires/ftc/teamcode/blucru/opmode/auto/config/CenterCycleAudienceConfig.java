package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterPreloadDeposit;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceCenterPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.StackToBackdropAudienceCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceClosePreloadDeposit;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceClosePreloadIntake;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadDeposit;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadIntakeForCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropToStackCenter;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterDepositFailsafe;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterIntakeCenterStack;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterIntakeFailsafe;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositCenterBackstage;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.DepositCenterCycle;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.CenterIntakeFarStack;
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
        DEPOSITING_BACKSTAGE,
        PARKING,
        DONE,

        // Failsafe states
        DEPOSIT_FAILSAFE,
        CRASH_FAILSAFE,
        INTAKE_FAILSAFE,
    }

    Randomization randomization;

    HashMap<Randomization, Path> preloadIntakePaths;
    HashMap<Randomization, Path> preloadDepositPaths;
    Path backdropToStackPath, stackToBackdropPath,
            intakeFarPath, intakeFarAfterFailedPath,
            intakeCenterStackPath,
            depositPath, depositBackstagePath,
            parkPath,

            toBackdropPreloadPath,

            depositFailsafePath, intakeFailsafePath,
            crashTrussBackdropFailsafePath, crashTrussStackFailsafePath,
            crashTrussMiddleFailsafeToIntakePath, crashTrussMiddleFailsafeToBackdropPath,
            crashToStackRecoveryPath, crashToBackdropRecoveryPath;

    Path currentPath;
    ElapsedTime runtime;
    StateMachine stateMachine;
    Drivetrain dt;
    Robot robot;

    public CenterCycleAudienceConfig() {
        preloadIntakePaths = new HashMap<>();
        preloadDepositPaths = new HashMap<>();
        stateMachine = new StateMachineBuilder()
                .state(State.PRELOAD_INTAKING)
                .onEnter(() -> logTransitionTo(State.PRELOAD_INTAKING))
                .transition(() -> currentPath.isDone(), State.TO_BACKDROP_PRELOAD, () -> {
                    currentPath = toBackdropPreloadPath.start();
                })
                .transition(() -> robot.intake.isFull(), State.TO_BACKDROP_PRELOAD, () -> {
                    currentPath = toBackdropPreloadPath.start();
                    Globals.stackFarPixels -= 1;
                })

                // TODO: create failsafe for if intake fails

                // TO BACKDROP PRELOAD
                .state(State.TO_BACKDROP_PRELOAD)
                .onEnter(() -> logTransitionTo(State.TO_BACKDROP_PRELOAD))
                .transition(() -> currentPath.isDone(), State.WAITING_FOR_PRELOAD_DEPOSIT)

                // WAITING FOR PRELOAD DEPOSIT
                .state(State.WAITING_FOR_PRELOAD_DEPOSIT)
                .onEnter(() -> logTransitionTo(State.WAITING_FOR_PRELOAD_DEPOSIT))
                .loop(() -> {
                    dt.updateAprilTags();
                })
                .transition(() -> robot.cvMaster.numDetections > 2, State.PRELOAD_DEPOSITING, () -> {
                    currentPath = preloadDepositPaths.get(randomization).start();
                })
                .transitionTimed(3, State.PRELOAD_DEPOSITING, () -> {
                    currentPath = preloadDepositPaths.get(randomization).start();
                })

                .state(State.PRELOAD_DEPOSITING)
                .onEnter(() -> logTransitionTo(State.PRELOAD_DEPOSITING))
                .loop(() -> dt.updateAprilTags())
                .transition(() -> currentPath.isDone(), State.TO_STACK, () -> {
                    currentPath = backdropToStackPath.start();
                })

                .state(State.TO_STACK)
                .onEnter(() -> {
                    logTransitionTo(State.TO_STACK);
                    robot.intake.startReadingColor();
                })
                .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                    if(Globals.stackFarPixels > 1) currentPath = intakeFarPath.start();
                    else currentPath = intakeCenterStackPath.start();
                })

                .state(State.INTAKING)
                .onEnter(() -> {
                    logTransitionTo(State.INTAKING);
                })
                .transition(() -> currentPath.isDone(), State.INTAKE_FAILSAFE, () -> {
                    currentPath = intakeFailsafePath.start();
                })
                .transition(() -> Robot.getInstance().intake.isFull(), State.TO_BACKDROP, () -> {
                    CommandScheduler.getInstance().cancelAll();
                    currentPath = stackToBackdropPath.start();
                    Robot.getInstance().outtake.lock();
                    Globals.stackFarPixels -= 2;
                })
                .transition(() -> runtime.seconds() > 27.0, State.TO_BACKDROP, () -> {
                    currentPath = stackToBackdropPath.start();
                    Robot.getInstance().outtake.lock();
                })

                .state(State.TO_BACKDROP)
                .onEnter(() -> {
                    logTransitionTo(State.TO_BACKDROP);
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() < 27.5, State.DEPOSITING, () -> {
                    currentPath = depositPath.start();
                })
                .transition(() -> currentPath.isDone() && (runtime.seconds() > 27.5 && runtime.seconds() < 29), State.DEPOSITING_BACKSTAGE, () -> {
                    currentPath = depositBackstagePath.start();
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() > 29, State.PARKING, () -> {
                    currentPath = parkPath.start();
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
                    currentPath = backdropToStackPath.start();
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() >= 25, State.PARKING, () -> {
                    new OuttakeRetractCommand(1.5).schedule();
                    currentPath = parkPath.start();
                })

                .state(State.INTAKE_FAILSAFE)
                .onEnter(() -> {
                    logTransitionTo(State.INTAKE_FAILSAFE);
                })
                .transition(() -> currentPath.isDone(), State.INTAKING, () -> {
                    currentPath = intakeFarAfterFailedPath.start();
                })
                .transition(() -> runtime.seconds()>27, State.TO_BACKDROP, () -> {
                    currentPath = stackToBackdropPath.start();
                })
                .transition(() -> Robot.getInstance().intake.isFull(), State.TO_BACKDROP, () -> {
                    CommandScheduler.getInstance().cancelAll();
                    currentPath = stackToBackdropPath.start();
                    Robot.getInstance().outtake.lock();
                    Globals.stackFarPixels -= 2;
                })

                .state(State.DEPOSITING_BACKSTAGE)
                .onEnter(() -> logTransitionTo(State.DEPOSITING_BACKSTAGE))
                .transition(() -> currentPath.isDone(), State.DONE)

                .state(State.PARKING)
                .onEnter(() -> logTransitionTo(State.PARKING))
                .transition(() -> currentPath.isDone(), State.DONE)

                .state(State.DONE)
                .build();
    }

    public void build() {
        preloadIntakePaths.put(Randomization.CLOSE, new AudienceClosePreloadIntake().build());
        preloadIntakePaths.put(Randomization.CENTER, new AudienceCenterPreloadIntakeForCenter().build());
        preloadIntakePaths.put(Randomization.FAR, new AudienceFarPreloadIntakeForCenter().build());

        toBackdropPreloadPath = new StackToBackdropAudienceCenterPreload().build();

        preloadDepositPaths.put(Randomization.CLOSE, AudienceClosePreloadDeposit.get().build());
        preloadDepositPaths.put(Randomization.CENTER, AudienceCenterPreloadDeposit.get().build());
        preloadDepositPaths.put(Randomization.FAR, AudienceFarPreloadDeposit.get().build());

        backdropToStackPath = new BackdropToStackCenter().build();
        stackToBackdropPath = new StackToBackdropCenter().build();
        intakeFarPath = new CenterIntakeFarStack().build();
        intakeFarAfterFailedPath = new CenterIntakeFarStack(0, 2, 30).build();

        intakeCenterStackPath = new CenterIntakeCenterStack().build();
        depositPath = new DepositCenterCycle().build();
        depositBackstagePath = new DepositCenterBackstage().build();
        parkPath = new PIDPathBuilder().addMappedPoint(42, 10, 220).build();

        intakeFailsafePath = new CenterIntakeFailsafe().build();
        depositFailsafePath = new CenterDepositFailsafe().build();
    }

    public void start(Randomization randomization) {
        dt = Robot.getInstance().drivetrain;
        robot = Robot.getInstance();
        this.randomization = randomization;
        currentPath = preloadIntakePaths.get(randomization);
        stateMachine.start();
        stateMachine.setState(State.PRELOAD_INTAKING);
        runtime = Globals.runtime;
    }

    public void run() {
        stateMachine.update();
        currentPath.run();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Audience Center Cycle");
        telemetry.addData("State", stateMachine.getState());
    }

    public void logTransitionTo(Enum to) {
        Log.i("CenterCycleAudienceConfig", "State transitioning to " + to);
    }
}
