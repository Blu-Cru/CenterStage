package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.IntakePowerCommand;

public class StopIntakeCommand extends SequentialCommandGroup {
    public StopIntakeCommand() {
        super(
                new SequentialCommandGroup(
                        new IntakePowerCommand(-1),
                        new WaitCommand(500),
                        new IntakePowerCommand(0),
                        new DropdownRetractCommand()
                )
        );
    }
}
