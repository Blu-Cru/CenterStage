package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeWristBackstageCommand extends InstantCommand {
    public OuttakeWristBackstageCommand() {
        super(
                () -> {
                     Robot.getInstance().outtake.wristBackstage();
                }
        );
    }
}
