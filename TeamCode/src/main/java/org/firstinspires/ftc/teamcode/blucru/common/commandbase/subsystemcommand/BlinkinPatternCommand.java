package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BlinkinPatternCommand extends InstantCommand {
    public BlinkinPatternCommand(RevBlinkinLedDriver.BlinkinPattern pattern) {
        super(
                () -> Robot.getInstance().blinkin.setPattern(pattern)
        );
    }
}
