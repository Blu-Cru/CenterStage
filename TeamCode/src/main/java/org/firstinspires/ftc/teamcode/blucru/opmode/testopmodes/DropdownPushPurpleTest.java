package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Dropdown Push Purple Test", group = "test")
public class DropdownPushPurpleTest extends BCLinearOpMode {
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
            gamepad1.rumble(1);
        }

        if(stickyG1.a) {
            intake.intakeWrist.dropToPurpleHeight();
        }

        if(stickyG1.b) {
            intake.intakeWrist.retract();
        }

        if(gamepad1.right_trigger > 0.1) intake.setIntakePower(gamepad1.right_trigger);
        else if(gamepad1.left_trigger > 0.1) intake.setIntakePower(-gamepad1.left_trigger);
        else intake.setIntakePower(0);
    }
}
