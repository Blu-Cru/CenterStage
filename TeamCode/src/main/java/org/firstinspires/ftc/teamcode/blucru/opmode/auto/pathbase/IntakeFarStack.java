package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class IntakeFarStack extends PIDPathBuilder {
    public IntakeFarStack(double xIncrement, double yIncrement) {
        super();
        this.setPower(0.5)
                .schedule(new IntakeCommand(Globals.stackCenterPixels, 1))
                .addMappedPoint(Field.INTAKE_X - xIncrement, 10, 180, 2)
                .waitMillis(400)
                .setPower(0.35)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new DropdownCommand(Globals.stackCenterPixels-1),
                        new WaitCommand(400),
                        new DropdownCommand(0)
                    )
                )
                .addMappedPoint(Field.INTAKE_X, 20 + yIncrement, 160, 2.5)
                .waitMillis(700);
    }

    public IntakeFarStack() {
        this(0, 0);
    }
}
