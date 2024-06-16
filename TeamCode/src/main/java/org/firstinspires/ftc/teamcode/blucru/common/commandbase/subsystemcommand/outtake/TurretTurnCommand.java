package org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurretTurnCommand extends InstantCommand {
    public TurretTurnCommand(double angle) {
        super(
                () -> {
                    Robot.getInstance().outtake.setTurretAngle(angle);
                }
        );
    }
}
