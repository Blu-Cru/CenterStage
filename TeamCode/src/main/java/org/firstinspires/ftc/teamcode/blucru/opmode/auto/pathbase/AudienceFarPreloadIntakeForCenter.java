package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class AudienceFarPreloadIntakeForCenter extends PIDPathBuilder {
    public AudienceFarPreloadIntakeForCenter() {
        super();
        this.setPower(0.45)
                .addMappedPoint(-55, 42, 135,6)
                .addMappedPoint(-58, 30, 180, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new PurplePixelRetractCommand(),
                        new DropdownPartialRetractCommand()
                ))
                .addMappedPoint(-53, 21, 220,2)

                .waitMillis(350)
                .schedule(new IntakeCommand(4, 1))
                .addMappedPoint(Field.INTAKE_X, 12, 180, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new DropdownCommand(3),
                        new WaitCommand(400),
                        new DropdownCommand(0)
                ))
                .waitMillis(1000);
    }
}
