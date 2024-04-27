package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoState;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.ParkType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.IntakeTrajectories;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Trajectories;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name = "red far center strafing auto", group = "test")
public class RedFarCenterStrafingAuto extends BCLinearOpMode {
    AutoState state = AutoState.RUNNING;
    Poses poses;

    Trajectories trajectories;
    IntakeTrajectories intakeTrajectories;
    PreloadDeposits preloadDeposits;

    TrajectorySequence depositClose;
    TrajectorySequence purpleIntake;

    ArrayList<TrajectorySequence> trajectoryList = new ArrayList<>();
    int trajectoryIndex = 0;

    public void initialize() {
        enableFTCDashboard();
        addDrivetrain(false);
        addOuttake();
        addIntake();
        addPurplePixelHolder();

        trajectories = new Trajectories(Alliance.RED, Side.FAR, AutoType.PRELOAD, ParkType.NONE);

        Poses.setAlliance(Alliance.RED);
        intakeTrajectories = new IntakeTrajectories(1);
        preloadDeposits = new PreloadDeposits(1);

        depositClose = preloadDeposits.depositCloseFromStart(robot);
        purpleIntake = intakeTrajectories.placePurpleIntakeThroughCenterFromBackdropClose(robot);

//        trajectoryList.add(depositClose);
//        trajectoryList.add(purpleIntake);
        trajectoryList = trajectories.buildWingCenterTrajectoriesStrafing(robot);
    }

    public void onStart() {
        drivetrain.setPoseEstimate(Poses.WING_STARTING_POSE);
    }

    public void periodic() {
        switch(state) {
            case RUNNING:
                if(!drivetrain.isBusy()) {
                    if(trajectoryIndex == trajectoryList.size()) {
                        state = AutoState.STOP;
                        break;
                    } else {
                        drivetrain.followTrajectorySequenceAsync(trajectoryList.get(trajectoryIndex));
                        trajectoryIndex++;
                    }
                }

                // follow trajectory
                try {
                    drivetrain.updateTrajectory();
                } catch (Exception e) {
                    // if the trajectory is interrupted, stop the op mode
                    requestOpModeStop();
                }
                break;
            case STOP:
                drivetrain.driveScaled(0,0,0);
                break;
        }
    }

    public void telemetry() {
        telemetry.addData("STATE:", state);
        telemetry.addData("Running trajectory " + trajectoryIndex + " out of ", trajectoryList.size());
    }

    public void end() {
        Globals.START_POSE = drivetrain.getPoseEstimate();
    }
}