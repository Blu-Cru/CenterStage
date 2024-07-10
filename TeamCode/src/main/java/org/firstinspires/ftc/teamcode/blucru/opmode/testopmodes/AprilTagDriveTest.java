package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.AprilTagPoseGetter;
import org.firstinspires.ftc.teamcode.blucru.opmode.KLinearOpMode;

@Disabled
@TeleOp(name = "April tag drive test", group = "test")
public class AprilTagDriveTest extends KLinearOpMode {
    private enum State {
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

        state = State.DRIVER_CONTROL;

        drivetrain.fieldCentric = false;

        Poses.setAlliance(Alliance.RED);
    }

    public void onStart() {
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;

        if(gamepad1.a && !lastA) {
            state = State.DRIVE_TO_POSITION;
            drivetrain.pidTo(Poses.DEPOSIT_CENTER_POSE);
            cvMaster.stop();
        }
        lastA = gamepad1.a;

        if(gamepad1.b && !lastB) {
            state = State.DRIVER_CONTROL;
            drivetrain.drivetrainState = DrivetrainState.TELEOP;
            cvMaster.detectTag();
        }
        lastB = gamepad1.b;

        switch(state) {
            case DRIVER_CONTROL:
                drivetrain.driveScaled(horz, vert, rot);

                if(gamepad1.right_stick_button) drivetrain.resetHeading(Math.toRadians(180));

                drivetrain.setPoseEstimate(AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(cvMaster.tagDetector.getDetections(), drivetrain.getHeading()));
                break;
            case DRIVE_TO_POSITION:
                break;
        }

        drivetrain.ftcDashDrawCurrentPose();
    }
}
