package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.outtake.Lock;

public class UnlockAllCommand extends SequentialCommandGroup {
    public UnlockAllCommand() {
        super(
                new ConditionalCommand(
                        new LockReleaseCommand(2),
                        new SequentialCommandGroup(
                                new LockReleaseCommand(1),
                                new WaitCommand(150),
                                new LockReleaseCommand(2)
                        ),
                        () -> Robot.getInstance().outtake.lock.lockState != Lock.LockState.LOCKED_FRONT
                )
        );
    }
}
