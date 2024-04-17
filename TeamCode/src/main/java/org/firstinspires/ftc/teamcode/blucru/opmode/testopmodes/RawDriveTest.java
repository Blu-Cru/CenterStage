package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.BCLinearOpMode;

@TeleOp(name = "raw drive test", group = "test")
public class RawDriveTest extends BCLinearOpMode {
    double vert, horz, rotate;
    @Override
    public void initialize() {
        addDrivetrain(true);
        addIntakeWrist();
        drivetrain.fieldCentric = false;
    }

    @Override
    public void periodic() {
        vert = -gamepad1.left_stick_y;
        horz = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        drivetrain.drive(horz, vert, rotate);
//        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(horz, vert), rotate));
    }

    @Override
    public void telemetry() {
        drivetrain.testTelemetry(telemetry);
    }
}
