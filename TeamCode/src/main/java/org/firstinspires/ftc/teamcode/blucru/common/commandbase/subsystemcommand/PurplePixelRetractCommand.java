package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class PurplePixelRetractCommand extends InstantCommand {
    public PurplePixelRetractCommand(boolean right) {
        super(() -> {
            if (right) Robot.getInstance().purplePixelHolder.release(-Globals.reflect);
            else Robot.getInstance().purplePixelHolder.release(Globals.reflect);
        });
    }

    public PurplePixelRetractCommand() {
        this(true);
    }
}
