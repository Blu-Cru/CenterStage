package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lock;

public class LockCommand extends InstantCommand {
    public LockCommand() {
        super(
                () -> {
                    Robot.getInstance().outtake.lock();
                }
        );
    }
}
