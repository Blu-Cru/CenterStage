package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.trajectories.Poses;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Drive test", group = "test")
public class DriveTest extends BluLinearOpMode {
    double vert, horz, rotate;

    @Override
    public void initialize() {
        addDrivetrain(true);
        drivetrain.drivePower = 0.8;
        addDropdown();
        enableFTCDashboard();
        Poses.setAlliance(Alliance.RED);
    }

    @Override
    public void periodic() {
        vert = -gamepad1.left_stick_y;
        horz = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) {
            drivetrain.setPoseEstimate(Poses.AUDIENCE_STARTING_POSE);
            drivetrain.resetHeading(Poses.AUDIENCE_STARTING_POSE.getHeading());
            gamepad1.rumble(150);
        }

        if(gamepad1.b) {
            if(gamepad1.left_bumper) {
//                    drivetrain.driveToDistanceToHeading(horz, vert, 10, Math.toRadians(180));
            } else {
                drivetrain.driveToHeadingScaled(horz, vert, Math.toRadians(180));
            }
        } else if (gamepad1.x) {
            if(gamepad1.left_bumper) {
//                    drivetrain.driveToDistanceToHeading(horz, vert, 10, 0);
            } else {
                drivetrain.driveToHeadingScaled(horz, vert, 0);
            }
        } else {
            drivetrain.teleOpDrive(horz, vert, rotate);
        }
        drivetrain.ftcDashDrawCurrentPose();
    }

    @Override
    public void telemetry() {
        drivetrain.testTelemetry(telemetry);
    }
}
