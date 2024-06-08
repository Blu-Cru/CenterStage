package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LiftRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeIncrementCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.TurretTurnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeRetractCommand extends SequentialCommandGroup {
    public OuttakeRetractCommand() {
        this(1);
    }

    public OuttakeRetractCommand(double incrementPixelHeight) {
        super(
                new ConditionalCommand(
                        new LiftRetractCommand(),
                        new SequentialCommandGroup(
                                new OuttakeIncrementCommand(incrementPixelHeight),
                                new WaitCommand(70),
                                new TurretTurnCommand(270),
                                new WaitCommand(200),
                                new OuttakeWristCommand(true),
                                new WaitCommand(300),
                                new LiftRetractCommand()
                        ),
                        () -> Robot.getInstance().outtake.wristRetracted
                )
        );
    }
}
