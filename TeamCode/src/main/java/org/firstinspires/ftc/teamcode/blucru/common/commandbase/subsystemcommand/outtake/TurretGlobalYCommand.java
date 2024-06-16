package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurretGlobalYCommand extends InstantCommand {
    public TurretGlobalYCommand(double yInches) {
        super(
                () -> Robot.getInstance().outtake.setTurretGlobalY(yInches * Globals.reflect)
        );
    }
}
