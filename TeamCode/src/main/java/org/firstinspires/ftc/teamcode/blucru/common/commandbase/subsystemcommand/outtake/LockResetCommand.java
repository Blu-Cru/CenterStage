package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class LockResetCommand extends InstantCommand {
    public LockResetCommand () {
        super(
                () -> Robot.getInstance().outtake.resetLock()
        );
    }
}
