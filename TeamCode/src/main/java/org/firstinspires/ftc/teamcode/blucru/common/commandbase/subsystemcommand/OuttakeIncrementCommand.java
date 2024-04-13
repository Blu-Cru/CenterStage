package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeIncrementCommand extends InstantCommand {
    public OuttakeIncrementCommand(double pixels) {
        super(
                () -> Robot.getInstance().outtake.incrementTargetHeight(pixels)
        );
    }
}
