package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.states.DrivetrainState;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@Config
@TeleOp(name="Drive To Position Test", group="test")
public class DriveToPositionTest extends BCLinearOpMode {
    public static double p = 0, i = 0, d = 0;
    public static double targetX = 0, targetY = 0, targetHeading = Math.toRadians(90);
    public static double power = 0.5;

    String mode = "driver control";

    public void initialize() {
        addDrivetrain(true);
        addDropdown();
        enableFTCDashboard();

        targetX = drivetrain.getPoseEstimate().getX();
        targetY = drivetrain.getPoseEstimate().getY();
        targetHeading = drivetrain.getPoseEstimate().getHeading();
    }

    public void periodic() {
        drivetrain.translationPID.setPID(p, i, d);
        drivetrain.drivePower = power;

        if(gamepad1.a) {
            mode = "driver control";
        }

        if(gamepad1.b) {
            mode = "drive to position";
        }

        if(mode.equals("driver control")) {
            drivetrain.drivetrainState = DrivetrainState.TELEOP;

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading(Math.toRadians(90));
                gamepad1.rumble(100);
            }

            drivetrain.teleOpDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        } else if(mode.equals("drive to position")) {
            drivetrain.pidTo(new Pose2d(targetX, targetY, targetHeading));
        }
    }

    public void telemetry() {
        telemetry.addData("mode", mode);
        telemetry.addData("target x", targetX);
        telemetry.addData("target y", targetY);
        telemetry.addData("target heading", targetHeading);
        telemetry.addData("current x", drivetrain.getPoseEstimate().getX());
        telemetry.addData("current y", drivetrain.getPoseEstimate().getY());
        telemetry.addData("current heading", drivetrain.getPoseEstimate().getHeading());
    }
}
