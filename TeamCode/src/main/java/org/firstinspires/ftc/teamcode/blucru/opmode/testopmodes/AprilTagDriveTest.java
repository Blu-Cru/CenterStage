package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import static org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes.AprilTagDriveTest.State.DRIVER_CONTROL;

import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.util.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

public class AprilTagDriveTest extends BCLinearOpMode {
    enum State {
        DRIVER_CONTROL,
        DRIVE_TO_POSITION
    }

    boolean lastA = false;
    boolean lastB = false;
    State state;

    @Override
    public void initialize() {
        addDrivetrain(true);
        addCVMaster();
        enableFTCDashboard();
    }

    public void onStart() {
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;
        drivetrain.teleOpDrive(horz, vert, rot);

        if(gamepad1.a && !lastA) {
            state = State.DRIVE_TO_POSITION;
            drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE);
        }
        lastA = gamepad1.a;

        if(gamepad1.b) {
            state = DRIVER_CONTROL;
            drivetrain.drivetrainState = DrivetrainState.TELEOP;
        }
        lastB = gamepad1.b;

        switch(state) {
            case DRIVER_CONTROL:
                drivetrain.setPoseEstimate(AprilTagLocalizer.getRobotPose(cvMaster.tagDetector.getDetections()));
                break;
            case DRIVE_TO_POSITION:
                break;
        }

        drivetrain.ftcDashDrawPose();
    }
}
