package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class CenterIntakeCenterStack extends PIDPathBuilder {
    public CenterIntakeCenterStack() {
        super();
        this.setPower(0.4)
                .schedule(new SequentialCommandGroup(
                        new IntakeCommand(Globals.stackCenterPixels-1),
                        new LockResetCommand()
                ))
                .addMappedPoint(Field.INTAKE_X, 12, 160, 2.5)
                .addMappedPoint(Field.INTAKE_X-0.5, 20, 160, 2.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new IntakeCommand(Globals.stackCenterPixels-2),
                        new WaitCommand(300),
                        new DropdownCommand(0)
                ))
                .waitMillis(900);
    }
}
