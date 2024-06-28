package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class CenterIntakeFailsafe extends PIDPathBuilder {
    public CenterIntakeFailsafe() {
        super();
        this.setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new DropdownPartialRetractCommand(),
                        new IntakePowerCommand(-1)
                ))
                .addMappedPoint(-45, 12, 180,6);
    }
}
