package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class DropdownFullRetractCommand extends InstantCommand {
    public DropdownFullRetractCommand() {
        super(
                () -> Robot.getInstance().intake.intakeWrist.retract()
        );
    }
}