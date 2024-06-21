package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeWristCommand extends InstantCommand {
    public OuttakeWristCommand(boolean retracted) {
        super(
                () -> {
                    if(retracted) Robot.getInstance().outtake.wristRetract();
                    else Robot.getInstance().outtake.wristExtend();
                }
        );
    }
}
