package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Dropdown Push Purple Test", group = "test")
public class DropdownPushPurpleTest extends BCLinearOpMode {
    @Override
    public void initialize() {
        addDrivetrain(true);
        addIntakeWrist();
    }

    @Override
    public void periodic() {
        drivetrain.teleOpDrive(gamepad1);

        if(gamepad1.right_stick_button) {
            drivetrain.resetHeading(Math.toRadians(90));
            gamepad1.rumble(1);
        }

        if(stickyG1.a) {
            intakeWrist.dropToPurpleHeight();
        }

        if(stickyG1.b) {
            intakeWrist.retract();
        }
    }
}
