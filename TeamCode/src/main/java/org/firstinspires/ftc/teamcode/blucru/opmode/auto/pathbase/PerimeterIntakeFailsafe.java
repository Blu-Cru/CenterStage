package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class PerimeterIntakeFailsafe extends PIDPathBuilder {
    public PerimeterIntakeFailsafe() {
        super();
        this.setPower(0.6)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new DropdownPartialRetractCommand(),
                        new IntakePowerCommand(-1)
                ))
                .addMappedPoint(-48, 50, 170,6);
    }
}
