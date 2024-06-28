package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;

public class StackToBackdropCenter extends PIDPathBuilder {
    public StackToBackdropCenter() {
        this(false);
    }

    public StackToBackdropCenter(boolean lastTrip) {
        super();
        this.setPower(0.7)
                .schedule(new SequentialCommandGroup(
                        new IntakePowerCommand(1)
                ))
                .addMappedPoint(-50, 12, 180, 4)
                .setPower(0.9)
                .schedule(new ConditionalCommand(
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new LockCommand(),
                                new IntakeStopCommand()
                        ),
                        new SequentialCommandGroup(
                                new IntakePowerCommand(-1),
                                new WaitCommand(800),
                                new LockCommand(),
                                new IntakeStopCommand()
                        ),
                        () -> lastTrip
                ))
                .addMappedPoint(10, 12, 180, 6);
    }
}
