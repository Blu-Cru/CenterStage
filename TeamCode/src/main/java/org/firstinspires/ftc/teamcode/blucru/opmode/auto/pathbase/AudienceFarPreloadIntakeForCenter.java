package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.PurplePixelRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class AudienceFarPreloadIntakeForCenter extends PIDPathBuilder {
    public AudienceFarPreloadIntakeForCenter() {
        super();
        this.setPower(0.45)
                .addMappedPoint(-54, 42, 135,6)
                .addMappedPoint(-57, 30, 180, 3)
                .schedule(new SequentialCommandGroup(
                        new DropdownPartialRetractCommand()
                ))
                .addMappedPoint(-52, 21, 220,3)
                .schedule(new SequentialCommandGroup(
                        new PurplePixelRetractCommand(),
                        new IntakeCommand(4, 1),
                        new LockResetCommand()
                ))
                .waitMillis(200)

                .addMappedPoint(-55.5, 17.5, 200, 3)
                .waitMillis(500)
                .schedule(new DropdownCommand(3))
                .waitMillis(600)
                .schedule(new DropdownCommand(0))
                .waitMillis(300);
    }
}
