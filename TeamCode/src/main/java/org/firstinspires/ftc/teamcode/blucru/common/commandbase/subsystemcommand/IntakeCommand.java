package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(double power) {
        super(
                () -> Robot.getInstance().intake.setIntakePower(power)
        );
    }
}
