package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import static org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes.AprilTagDriveTest.State.DRIVER_CONTROL;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.common.util.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name = "april tag drive test", group = "test")
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

        state = DRIVER_CONTROL;

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
            drivetrain.lockTo(Poses.DEPOSIT_CENTER_POSE);
            cvMaster.stop();
        }
        lastA = gamepad1.a;

        if(gamepad1.b && !lastB) {
            state = DRIVER_CONTROL;
            drivetrain.drivetrainState = DrivetrainState.TELEOP;
            cvMaster.detectTag();
        }
        lastB = gamepad1.b;

        switch(state) {
            case DRIVER_CONTROL:
                drivetrain.drive(horz, vert, rot);

                if(gamepad1.right_stick_button) drivetrain.resetHeading(Math.toRadians(180));


                drivetrain.setPoseEstimate(AprilTagLocalizer.getRobotPose(cvMaster.tagDetector.getDetections()));
                break;
            case DRIVE_TO_POSITION:
                break;
        }

        drivetrain.ftcDashDrawPose();
    }
}
