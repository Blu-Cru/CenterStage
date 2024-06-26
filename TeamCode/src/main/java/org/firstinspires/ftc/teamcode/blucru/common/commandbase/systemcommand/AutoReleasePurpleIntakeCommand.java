package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class AutoReleasePurpleIntakeCommand extends SequentialCommandGroup {
    public AutoReleasePurpleIntakeCommand(double waitMillis) {
        super(
                new WaitCommand((long) waitMillis),
                new InstantCommand(() -> {
                    Robot.getInstance().intake.startReleasePurple();
                })
        );
    }

    public AutoReleasePurpleIntakeCommand() {
        this(0);
    }
}
