package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Disabled
@TeleOp(name = "imu drive test", group = "test")
public class ImuUpdateTest extends BCLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain(true);
        enableFTCDashboard();
    }

    @Override
    public void periodic() {
        if(stickyG1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
        }

        if(stickyG1.left_stick_button) {
            gamepad1.rumble(150);
            drivetrain.fusedLocalizer.usingIMU = !drivetrain.fusedLocalizer.usingIMU;
        }

        double horz = gamepad1.left_stick_x;
        double vert = -gamepad1.left_stick_y;
        double rot = -gamepad1.right_stick_x;

        if(gamepad1.b) {
            drivetrain.driveToHeadingScaled(horz, vert, Math.toRadians(180));
        } else {
            drivetrain.teleOpDrive(horz, vert, rot);
        }

        drivetrain.ftcDashDrawCurrentPose();
    }

    public void telemetry() {
        telemetry.addData("Using IMU", drivetrain.fusedLocalizer.usingIMU);
    }
}
