package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class IntakeFarStack extends PIDPathBuilder {
    public IntakeFarStack(int initialHeight, double xIncrement, double yIncrement) {
        super();
        this.setPower(0.6)
                .schedule(new IntakeCommand(initialHeight, 1))
                .addMappedPoint(Field.INTAKE_X - xIncrement, 12, 160, 2)
                .setPower(0.35)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(200),
                        new DropdownCommand(initialHeight-1),
                        new WaitCommand(200),
                        new DropdownCommand(0)
                    )
                )
                .addMappedPoint(Field.INTAKE_X, 20 + yIncrement, 160);
    }

    public IntakeFarStack(int initialHeight) {
        this(initialHeight, 0, 0);
    }
}
