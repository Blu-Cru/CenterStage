package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(int stackHeight) {
        super(
                new SequentialCommandGroup(
                        new DropdownCommand(stackHeight),
                        new WaitCommand(200),
                        new IntakePowerCommand(1)
                )
        );
    }
}
