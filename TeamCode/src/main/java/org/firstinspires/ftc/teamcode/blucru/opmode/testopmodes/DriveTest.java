package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Drive test", group = "test")
public class DriveTest extends BCLinearOpMode {
    double vert, horz, rotate;

    @Override
    public void initialize() {
        addDrivetrain(true);
        drivetrain.drivePower = 0.8;
        addIntakeWrist();
        enableFTCDashboard();
    }

    @Override
    public void periodic() {
        vert = -gamepad1.left_stick_y;
        horz = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
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
    }

    @Override
    public void telemetry() {
        drivetrain.testTelemetry(telemetry);
    }
}
