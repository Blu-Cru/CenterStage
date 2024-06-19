package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Dropdown Test", group = "test")
public class DropdownTest extends BCLinearOpMode {
    int stackHeight = 0;
    @Override
    public void initialize() {
        addDrivetrain(true);
        addIntake();
    }

    @Override
    public void periodic() {
        drivetrain.teleOpDrive(gamepad1);

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
            gamepad1.rumble(100);
        }

        if(stickyG1.a) {
            intake.dropdown.dropToStack(stackHeight);
        }

        if(stickyG1.b) {
            intake.dropdown.retract();
        }

        if(stickyG1.dpad_up) stackHeight = Math.min(5, stackHeight + 1);
        if(stickyG1.dpad_down) stackHeight = Math.max(0, stackHeight - 1);

        if(gamepad1.right_trigger > 0.1) intake.setIntakePower(gamepad1.right_trigger);
        else if(gamepad1.left_trigger > 0.1) intake.setIntakePower(-gamepad1.left_trigger);
        else intake.setIntakePower(0);
    }

    public void telemetry() {
        telemetry.addData("Stack Height", stackHeight);
        telemetry.addData("Wrist Angle", intake.dropdown.targetAngleDeg);
    }
}
