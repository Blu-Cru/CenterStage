package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class StackToBackdropPerimeter extends PIDPathBuilder {
    public StackToBackdropPerimeter() {
        super();
        this.setPower(0.7)
                .schedule(new SequentialCommandGroup(
                        new LockResetCommand(),
                        new IntakePowerCommand(1),
                        new WaitCommand(100),
                        new IntakeStopCommand()
                ))
                .addMappedPoint(-35, 60, 180, 6)
                .addMappedPoint(10, 60, 180, 6);
    }
}
