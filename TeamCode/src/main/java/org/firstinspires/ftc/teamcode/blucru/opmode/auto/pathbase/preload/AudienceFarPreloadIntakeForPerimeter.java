package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class AudienceFarPreloadIntakeForPerimeter extends PIDPathBuilder {
    public AudienceFarPreloadIntakeForPerimeter() {
        super();
        this.setPower(0.45)
                .addMappedPoint(-48, 48, 120,6)
                .addMappedPoint(-52, 41, 150, 2)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(20),
                        new PurplePixelRetractCommand(),
                        new DropdownPartialRetractCommand()
                ))
                .waitMillis(200)
                .schedule(new IntakeCommand(4, 1))
                .addMappedPoint(Field.INTAKE_X, 36, 180, 5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(200),
                        new DropdownCommand(3),
                        new WaitCommand(500),
                        new DropdownCommand(0)
                ))
                .waitMillis(1000);
    }
}
