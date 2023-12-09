package org.firstinspires.ftc.teamcode.BluCru.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BluCru.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

@TeleOp(name = "drive test", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    Drivetrain drivetrain;
    TwoWheelTrackingLocalizer localizer;

    double vert, horz, rotate;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, drivetrain);
        drivetrain.setDrivePower(0.5);

        localizer.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        drivetrain.init();

        waitForStart();
        while(opModeIsActive()) {
            vert = -gamepad1.left_stick_y;
            horz = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeadingOffset();
            }

            drivetrain.testDrive(new Vector2d(horz, vert), rotate);

            drivetrain.update();

            drivetrain.telemetry(telemetry);
            drivetrain.testTelemetry(telemetry);
            telemetry.addData("pose estimate", localizer.getPoseEstimate());
            telemetry.addData("pose velocity", localizer.getPoseVelocity());
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }
    }
}