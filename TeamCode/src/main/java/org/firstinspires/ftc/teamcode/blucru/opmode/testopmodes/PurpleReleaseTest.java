package org.firstinspires.ftc.teamcode.blucru.opmode.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.AutoReleasePurpleIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BCLinearOpMode;

@TeleOp(name = "Purple Release Test", group = "test")
public class PurpleReleaseTest extends BCLinearOpMode {
    @Override
    public void initialize() {
        addIntake();
    }

    @Override
    public void periodic() {
        if(stickyG1.a) new AutoReleasePurpleIntakeCommand(0).schedule();
    }
}
