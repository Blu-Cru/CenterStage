package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class DropdownPartialRetractCommand extends InstantCommand {
    public DropdownPartialRetractCommand() {
        super(
                () -> Robot.getInstance().intake.intakeWrist.dropToAutoMidPos()
        );
    }
}
