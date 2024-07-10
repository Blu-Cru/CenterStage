package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class CenterIntakeFarStack extends PIDPathBuilder {
    public CenterIntakeFarStack(int stackHeight, double xIncrement, double wiggleAngleDeg) {
        super();
        this.setPower(0.4)
                .schedule(new SequentialCommandGroup(
                        new LockResetCommand(),
                        new IntakePowerCommand(1)
                ))
                .addMappedPoint(Field.INTAKE_X - xIncrement, 12, 180, 3.5)
                .schedule(new SequentialCommandGroup(
                        new IntakeCommand(stackHeight),
                        new WaitCommand(150),
                        new IntakeCommand(stackHeight-1),
                        new WaitCommand(150),
                        new IntakeCommand(0)
                ))
                .waitMillis(450)
                .addMappedPoint(Field.INTAKE_X - xIncrement, 12, 180-wiggleAngleDeg, 2.5)
                .waitMillis(150)
                .addMappedPoint(Field.INTAKE_X - xIncrement, 12, 180 + wiggleAngleDeg, 2.5)
                .waitMillis(300);
    }

    public CenterIntakeFarStack() {
        this(Globals.stackCenterPixels-1, 0, 18);
    }

    public CenterIntakeFarStack(double xIncrement, double wiggleAngleDeg) {
        this(Globals.stackCenterPixels-1, xIncrement, wiggleAngleDeg);
    }
}
