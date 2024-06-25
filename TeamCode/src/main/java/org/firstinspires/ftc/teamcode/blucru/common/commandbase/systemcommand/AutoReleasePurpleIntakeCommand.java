package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class AutoReleasePurpleIntakeCommand extends SequentialCommandGroup {
    public AutoReleasePurpleIntakeCommand(double waitMillis, long outtakeTime) {
        super(
                new DropdownPartialRetractCommand(),
                new WaitCommand((long) waitMillis),
                new IntakePowerCommand(Globals.correctPower(-0.8)),
                new WaitCommand(outtakeTime),
                new IntakePowerCommand(0)
        );
    }

    public AutoReleasePurpleIntakeCommand(double waitMillis) {
        this(waitMillis, 200);
    }

    public AutoReleasePurpleIntakeCommand() {
        this(0);
    }
}
