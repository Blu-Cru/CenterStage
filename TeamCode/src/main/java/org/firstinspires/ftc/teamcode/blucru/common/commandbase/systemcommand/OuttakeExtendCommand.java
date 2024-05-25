package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeExtendCommand extends SequentialCommandGroup {
    public OuttakeExtendCommand(double pixelHeight) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> {
                                    Robot.getInstance().outtake.setTargetPixelHeight(pixelHeight);
                                }),
                        new WaitCommand(150),
                        new OuttakeWristCommand(false)
                )
        );
    }
}
