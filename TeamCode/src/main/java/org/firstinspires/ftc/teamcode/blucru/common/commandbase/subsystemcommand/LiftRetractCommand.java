package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class LiftRetractCommand extends InstantCommand {
    public LiftRetractCommand() {
        super(
                () -> Robot.getInstance().outtake.retractLift()
        );
    }
}
