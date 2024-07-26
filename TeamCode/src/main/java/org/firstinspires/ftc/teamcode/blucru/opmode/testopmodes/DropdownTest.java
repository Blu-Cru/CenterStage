package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Dropdown Test", group = "test")
public class DropdownTest extends BluLinearOpMode {
    int stackHeight = 0;
    @Override
    public void initialize() {
        addDropdown();
    }

    @Override
    public void periodic() {
//        drivetrain.teleOpDrive(gamepad1);
//
//        if(gamepad1.right_stick_button) {
//            drivetrain.resetHeading(Math.toRadians(90));
//            gamepad1.rumble(100);
//        }
//
        if(stickyG1.a) {
            dropdown.dropToStack(stackHeight);
        }

        if(stickyG1.b) {
            dropdown.retract();
        }

        if(stickyG1.dpad_up) stackHeight = Math.min(5, stackHeight + 1);
        if(stickyG1.dpad_down) stackHeight = Math.max(0, stackHeight - 1);
    }

    public void telemetry() {
        telemetry.addData("Stack Height", stackHeight);
    }
}
