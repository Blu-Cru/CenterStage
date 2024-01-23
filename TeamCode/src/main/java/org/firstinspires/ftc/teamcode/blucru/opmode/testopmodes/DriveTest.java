package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

@TeleOp(name = "drive test", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    Drivetrain drivetrain;
    Intake intake;
    TwoWheelTrackingLocalizer localizer;

    double vert, horz, rotate;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, drivetrain);

        localizer.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        drivetrain.init();
        intake.init();
        drivetrain.fieldCentric = true;
        drivetrain.drivePower = 0.5;

        waitForStart();
        while(opModeIsActive()) {
            drivetrain.read();
            intake.read();

            vert = Math.pow(-gamepad1.left_stick_y, 3);
            horz = Math.pow(gamepad1.left_stick_x, 3);
            rotate = Math.pow(-gamepad1.right_stick_x, 3);

            if(gamepad1.right_stick_button) {
                drivetrain.resetHeading();
                gamepad1.rumble(150);
            }

            if(gamepad1.b) {
                if(gamepad1.left_bumper) {
                    drivetrain.driveToDistanceToHeading(horz, vert, 10, Math.toRadians(180));
                } else {
                    drivetrain.driveToHeading(horz, vert, Math.toRadians(180));
                }
            } else if (gamepad1.x) {
                if(gamepad1.left_bumper) {
                    drivetrain.driveToDistanceToHeading(horz, vert, 10, 0);
                } else {
                    drivetrain.driveToHeading(horz, vert, 0);
                }
            } else {
                drivetrain.drive(horz, vert, rotate);
            }

            drivetrain.write();
            intake.write();
            drivetrain.testTelemetry(telemetry);
            drivetrain.telemetry(telemetry);
            telemetry.update();
        }
    }
}
