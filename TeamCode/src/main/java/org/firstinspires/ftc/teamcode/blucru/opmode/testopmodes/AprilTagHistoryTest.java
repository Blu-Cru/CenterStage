package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "april tag history", group = "test")
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
        Poses.setAlliance(Alliance.BLUE);
        drivetrain.fieldCentric = false;
    }

    @Override
    public void periodic() {
        double horz, vert, rot;

        switch(state) {
            case IDLE:

                if(gamepad1.a && !lastA) {
                    state = State.SCANNING;
                    cvMaster.detectTag();
                }
                lastA = gamepad1.a;

                horz = gamepad1.left_stick_x;
                vert = -gamepad1.left_stick_y;
                rot = -gamepad1.right_stick_x;

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

                horz = gamepad1.left_stick_x;
                vert = -gamepad1.left_stick_y;
                rot = -gamepad1.right_stick_x;

                drivetrain.teleOpDrive(horz, vert, rot);
                if(gamepad1.right_stick_button) {
                    drivetrain.resetHeading(Math.toRadians(90));
                }

                drivetrain.updateAprilTags(cvMaster.tagDetector);
                break;
            case DEPOSIT:
                if(drivetrain.isAtTargetPose() || gamepad1.a && !lastA) {
                    state = State.IDLE;
                    drivetrain.fieldCentric = false;
                }
                lastA = gamepad1.a;

                drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE);
                break;
        }

        drivetrain.ftcDashDrawCurrentPose();
    }

    public void telemetry() {
        telemetry.addData("State", state);
    }
}
