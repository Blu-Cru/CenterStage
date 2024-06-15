package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class AudienceClosePreloadIntake extends PIDPathBuilder {
    public AudienceClosePreloadIntake() {
        super();
        this.setPower(0.45)
                .addMappedPoint(-38, 45, 120,6)
                .addMappedPoint(-33, 36, 160, 2)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(20),
                        new PurplePixelRetractCommand()
                ))
                .waitMillis(200)
                .addMappedPoint(-44, 30, 160, 5)
                .schedule(new IntakeCommand(4, 1))
                .addMappedPoint(Field.INTAKE_X, 24, 180)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(200),
                        new DropdownCommand(3)
                ))
                .waitMillis(1000);
    }
}
