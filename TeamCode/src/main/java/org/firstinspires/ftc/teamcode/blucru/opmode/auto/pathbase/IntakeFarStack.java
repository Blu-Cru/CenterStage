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
        this.setPower(0.5)
                .schedule(new IntakeCommand(initialHeight, 1))
                .addMappedPoint(Field.INTAKE_X - xIncrement, 12, 160, 2)
                .setPower(0.35)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(500),
                        new DropdownCommand(initialHeight-1),
                        new WaitCommand(1000),
                        new DropdownCommand(0)
                    )
                )
                .addMappedPoint(Field.INTAKE_X, 20 + yIncrement, 160)
                .waitMillis(700);
    }

    public IntakeFarStack(int initialHeight) {
        this(initialHeight, 0, 0);
    }
}
