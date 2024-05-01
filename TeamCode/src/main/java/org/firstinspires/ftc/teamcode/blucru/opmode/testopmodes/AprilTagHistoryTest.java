package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

public class AprilTagHistoryTest extends BCLinearOpMode {
    enum State {
        IDLE,
        SCANNING,
        DEPOSIT
    }

    State state;

    boolean lastA = false;
    @Override
    public void initialize() {
        addDrivetrain(false);
        addCVMaster();
        state = State.IDLE;
        enableFTCDashboard();
    }

    @Override
    public void periodic() {
        switch(state) {
            case IDLE:

                if(gamepad1.a && !lastA) {
                    state = State.SCANNING;
                    cvMaster.detectTag();
                }
                lastA = gamepad1.a;

                double horz = gamepad1.left_stick_x;
                double vert = -gamepad1.left_stick_y;
                double rot = -gamepad1.right_stick_x;

                drivetrain.teleOpDrive(horz, vert, rot);
                if(gamepad1.right_stick_button) {
                    drivetrain.resetHeading(Math.toRadians(90));
                }

                break;
            case SCANNING:
                if(gamepad1.a && !lastA) {
                    state = State.DEPOSIT;
                    cvMaster.stop();
                    drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE);
                }
                lastA = gamepad1.a;

                drivetrain.updateAprilTags(cvMaster.tagDetector);
                break;
            case DEPOSIT:
                if(drivetrain.isAtTargetPose() || gamepad1.a && !lastA) {
                    state = State.IDLE;
                }
                lastA = gamepad1.a;
                break;
        }

        drivetrain.ftcDashDrawCurrentPose();
    }

    public void telemetry() {
        telemetry.addData("State", state);
    }
}
