package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class AudienceCenterPreloadIntakeForCenter extends PIDPathBuilder {
    public AudienceCenterPreloadIntakeForCenter() {
        super();
        this.setPower(0.45)
                .addMappedPoint(-40, 30, 135,6)
                .addMappedPoint(-48, 25, 180, 2)
                .schedule(new SequentialCommandGroup(
                        new PurplePixelRetractCommand(),
                        new DropdownPartialRetractCommand()
                ))
                .waitMillis(200)
                .schedule(new SequentialCommandGroup(
                        new IntakeCommand(4, 1),
                        new LockResetCommand()
                ))
                .addMappedPoint(Field.INTAKE_X, 12, 180, 3)
                .waitMillis(600)
                .schedule(new DropdownCommand(3))
                .waitMillis(600)
                .schedule(new DropdownCommand(0));
    }
}
