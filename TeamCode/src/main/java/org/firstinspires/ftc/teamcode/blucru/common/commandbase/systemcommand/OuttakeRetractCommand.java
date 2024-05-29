package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeIncrementCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeRetractCommand extends SequentialCommandGroup {
    public OuttakeRetractCommand() {
        super(
                new ConditionalCommand(
                        new LiftRetractCommand(),
                        new SequentialCommandGroup(
                                new OuttakeIncrementCommand(1),
                                new WaitCommand(300),
                                new OuttakeWristCommand(true),
                                new WaitCommand(200),
                                new LiftRetractCommand()
                        ),
                        () -> Robot.getInstance().outtake.wristRetracted
                )
        );
    }
}
