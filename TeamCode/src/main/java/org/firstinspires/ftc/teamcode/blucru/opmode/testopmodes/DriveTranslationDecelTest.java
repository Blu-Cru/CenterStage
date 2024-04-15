package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name = "Drive Translational Decel test", group = "test")
public class DriveTranslationDecelTest extends BCLinearOpMode {
    enum State {
        DRIVING,
        STOPPING,
        STOPPED
    }

    Pose2d startPose = new Pose2d(0, 0, 0);
    Pose2d endPose = new Pose2d(0, 0, 0);
    double startTimeSecs = 0;

    State state = State.DRIVING;

    @Override
    public void initialize() {
        addDrivetrain(true);
    }

    @Override
    public void periodic() {
        switch(state) {
            case DRIVING:
                double horz = gamepad1.left_stick_x;
                double vert = -gamepad1.left_stick_y;
                double rot = -gamepad1.right_stick_x;

                if(gamepad1.right_stick_button) {
                    drivetrain.resetHeading(Math.toRadians(90));
                }

                if(gamepad1.a) {
                    state = State.STOPPING;
                    startTimeSecs = currentSecs();
                    startPose = drivetrain.getPoseEstimate();
                }

                telemetry.addLine("Robot is driving, press a to start test");

                drivetrain.teleOpDrive(horz, vert, rot);
                break;
            case STOPPING:
                if(drivetrain.isStopped() && secsSince(startTimeSecs) > 1.5) {
                    state = State.STOPPED;
                    endPose = drivetrain.getPoseEstimate();
                    gamepad1.rumble(100); // rumble the controller to indicate the end of deceleration
                }
                telemetry.addLine("Robot is decelerating");
                drivetrain.setWeightedDrivePower(new Pose2d(0, 0, 0));
                break;
            case STOPPED:
                if(gamepad1.b) {
                    state = State.DRIVING; // reset the op mode
                    gamepad1.rumble(100); // rumble the controller to indicate the reset
                }

                telemetry.addLine("Robot has stopped, press b to reset");
                break;
        }

    }

    public void telemetry() {
        telemetry.addData("state:", state);
        telemetry.addData("start pose:", startPose);
        telemetry.addData("end pose:", endPose);
    }
}
