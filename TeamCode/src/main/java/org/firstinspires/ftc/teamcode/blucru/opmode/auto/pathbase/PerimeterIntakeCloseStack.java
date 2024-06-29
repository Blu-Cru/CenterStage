package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class PerimeterIntakeCloseStack extends PIDPathBuilder {
    public PerimeterIntakeCloseStack() {
        this(Globals.stackClosePixels -1, 0, 18);
    }

    public PerimeterIntakeCloseStack (int stackHeight, double xIncrement, double wiggleAngleDeg) {
        super();
        this.setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new IntakePowerCommand(1),
                        new LockResetCommand()
                ))
                .addMappedPoint(Field.INTAKE_X - xIncrement, 48, 200, 4)
                .setPower(0.35)
                .addMappedPoint(Field.INTAKE_X - xIncrement - 2.5, 42, 210, 2.5)
                .schedule(new SequentialCommandGroup(
                        new IntakeCommand(stackHeight),
                        new WaitCommand(400),
                        new IntakeCommand(stackHeight-1),
                        new WaitCommand(350),
                        new IntakeCommand(0)
                ))
                .waitMillis(700)
                .addMappedPoint(Field.INTAKE_X - xIncrement-2.5, 43, 210-wiggleAngleDeg, 2.5)
                .waitMillis(250)
                .addMappedPoint(Field.INTAKE_X - xIncrement-2.5, 43, 210 + wiggleAngleDeg, 2.5)
                .waitMillis(250);
    }
}
