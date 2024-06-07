package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeWristCommand extends InstantCommand {
    public OuttakeWristCommand(boolean retracted) {
        super(
                () -> Robot.getInstance().outtake.extendWrist()
        );
    }
}
