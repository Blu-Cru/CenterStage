package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class LockReleaseCommand extends InstantCommand {
    public LockReleaseCommand(int numReleased) {
        super(
                () -> {
                    Robot.getInstance().outtake.unlock(numReleased);
                }
        );
    }
}
