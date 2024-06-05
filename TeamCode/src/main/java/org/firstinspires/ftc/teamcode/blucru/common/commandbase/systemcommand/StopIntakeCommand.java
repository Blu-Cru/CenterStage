package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownFullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class StopIntakeCommand extends SequentialCommandGroup {
    public StopIntakeCommand() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new IntakePowerCommand(0),
                                new DropdownFullRetractCommand()
                        ),
                        new SequentialCommandGroup( // if intake was intaking, reverse it and retract the dropdown
                                new DropdownPartialRetractCommand(),
                                new IntakePowerCommand(-1),
                                new WaitCommand(600),
                                new IntakePowerCommand(0),
                                new DropdownFullRetractCommand()
                        ),

                        () -> Robot.getInstance().intake.getIntakePower() <= 0
                )
        );
    }
}
