package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeIncrementCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeWristCommand;

public class OuttakeRetractCommand extends SequentialCommandGroup {
    public OuttakeRetractCommand() {
        super(
                new SequentialCommandGroup(
                        new OuttakeIncrementCommand(1),
                        new WaitCommand(500),
                        new OuttakeWristCommand(true),
                        new WaitCommand(100),
                        new LiftRetractCommand()
                )
        );
    }
}
