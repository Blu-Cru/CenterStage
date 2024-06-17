package org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class OuttakeExtendCommand extends SequentialCommandGroup {
    /*
    PIXEL HEIGHT:
    0 leaves enough room for one pixel under the bucket
    -1 is the lowest possible height for the backdrop
    each increment is 1 pixel height up the board
     */
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
