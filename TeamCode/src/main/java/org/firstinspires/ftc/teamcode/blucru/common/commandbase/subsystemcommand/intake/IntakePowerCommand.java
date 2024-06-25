package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class IntakePowerCommand extends InstantCommand {
    public IntakePowerCommand(double power) {
        super(
                () -> Robot.getInstance().intake.setPower(power)
        );
    }
}
