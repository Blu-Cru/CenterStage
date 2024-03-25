package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.states.AutoState;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.IntakeTrajectories;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.PreloadDeposits;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name = "deposit and placement test", group = "test")
public class DepositAndPlacementTest extends BCLinearOpMode {
    AutoState state = AutoState.RUNNING;
    Poses poses;
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
        addPurplePixelHolder();

        poses = new Poses(1);
        intakeTrajectories = new IntakeTrajectories(1);
        preloadDeposits = new PreloadDeposits(1);

        depositClose = preloadDeposits.depositCloseFromStart(robot);
        purpleIntake = intakeTrajectories.placePurpleIntakeThroughCenterFromBackdropClose(robot);

        trajectoryList.add(depositClose);
        trajectoryList.add(purpleIntake);
    }

    public void onStart() {
        drivetrain.setPoseEstimate(Poses.BACKDROP_STARTING_POSE);
    }

    public void periodic() {
        switch(state) {
            case RUNNING:
                if(!drivetrain.isBusy()) {
                    if(trajectoryIndex >= trajectoryList.size()) {
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
                drivetrain.drive(0,0,0);
                break;
        }
    }

    public void telemetry() {
        telemetry.addData("STATE:", state);
        telemetry.addData("Running trajectory " + trajectoryIndex + " out of ", trajectoryList.size());
    }
}
