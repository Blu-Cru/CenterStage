package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class AudienceCenterPreloadIntakeForPerimeter extends PIDPathBuilder {
    public AudienceCenterPreloadIntakeForPerimeter() {
        super();
        this.setPower(0.45)
                .addMappedPoint(-40, 30, 135,6)
                .addMappedPoint(-48, 25, 180, 2)
                .schedule(new SequentialCommandGroup(
                        new PurplePixelRetractCommand(),
                        new DropdownPartialRetractCommand()
                ))
                .waitMillis(200)
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
