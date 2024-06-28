package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class PerimeterIntakeCloseStack extends PIDPathBuilder {
    public PerimeterIntakeCloseStack () {
        super();
        this.setPower(0.4)
                .schedule(new SequentialCommandGroup(
                        new IntakeCommand(Globals.stackClosePixels-1),
                        new LockResetCommand()
                ))
                .addMappedPoint(Field.INTAKE_X, 44, 160, 2.5)
                .setPower(0.35)
                .addMappedPoint(Field.INTAKE_X-0.5, 40, 160, 2.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new IntakeCommand(Globals.stackClosePixels-2),
                        new WaitCommand(300),
                        new DropdownCommand(0)
                ))
                .waitMillis(900);
    }
}
