package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class StackToBackdropCenter extends PIDPathBuilder {
    public StackToBackdropCenter() {
        super();
        this.setPower(0.7)
                .schedule(new SequentialCommandGroup(
                        new IntakePowerCommand(1),
                        new WaitCommand(100),
                        new IntakeStopCommand()
                ))
                .addMappedPoint(-50, 12, 180, 6)
                .addMappedPoint(10, 12, 180, 6);
    }
}
