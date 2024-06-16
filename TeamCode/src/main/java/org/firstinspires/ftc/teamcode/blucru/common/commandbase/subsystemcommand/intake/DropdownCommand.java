package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class DropdownCommand extends InstantCommand {
    public DropdownCommand(int stackHeight) {
        super(
                () -> Robot.getInstance().intake.intakeWrist.dropToStack(stackHeight)
        );
    }
}
