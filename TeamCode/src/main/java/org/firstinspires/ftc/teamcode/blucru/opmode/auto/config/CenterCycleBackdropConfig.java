package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.util.Utils;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropCenterPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropClosePreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropFarPreload;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.BackdropThroughTrussCenter;

import java.util.HashMap;

public class CenterCycleBackdropConfig extends AutoConfig {
    static final Pose2d START_POSE = Utils.mapPose(12, 61, Math.toRadians(90));
    static final double TRUSS_HEADING_FAILSAFE_TOLERANCE = Math.toRadians(40);

    Path farPreloadPath, centerPreloadPath, closePreloadPath;
    Path backdropToStackPath, stackToBackdropPath;
    Path backdropFailsafePath, stackFailsafePath;
    Path crashTrussBackdropFailsafePath, crashTrussStackFailsafePath, crashTrussMiddleFailsafePath;
    Path crashToStackRecoveryPath, crashToBackdropRecoveryPath;

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
                Log.i("CenterCycleConfig", "looped");
                dt.logPose();
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
//                dt.intake.start();
            })
            .transition(() -> dt.getAbsHeadingError(Math.PI) > TRUSS_HEADING_FAILSAFE_TOLERANCE, State.CRASH_TO_STACK_FAILSAFE, () -> {
                if(dt.pose.getX() > -8) currentPath = crashTrussBackdropFailsafePath.start();
                else currentPath = crashTrussMiddleFailsafePath.start();
            })

            // CRASH FAILSAFE STATE

            .state(State.CRASH_TO_STACK_FAILSAFE)
//            .transition(currentPath::isDone, State.TO_STACK, () -> {
//                currentPath = crashToStackRecoveryPath.start();
//            })

            .build();

    public CenterCycleBackdropConfig() {

    }

    public void build() {
        dt = Robot.getInstance().drivetrain;

        preloadPaths.put(Randomization.FAR, new BackdropFarPreload().build());
        preloadPaths.put(Randomization.CENTER, new BackdropCenterPreload().build());
        preloadPaths.put(Randomization.CLOSE, new BackdropClosePreload().build());

        backdropToStackPath = new BackdropThroughTrussCenter().build();

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
