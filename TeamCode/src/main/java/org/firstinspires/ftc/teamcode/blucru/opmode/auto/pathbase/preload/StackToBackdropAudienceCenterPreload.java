package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownFullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockResetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class StackToBackdropAudienceCenterPreload extends PIDPathBuilder {
    public StackToBackdropAudienceCenterPreload() {
        super();
        this.setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new LockResetCommand(),
                        new IntakePowerCommand(1),
                        new DropdownFullRetractCommand(),
                        new WaitCommand(100),
                        new LockCommand(),
                        new IntakePowerCommand(-1),
                        new WaitCommand(800),
                        new IntakeStopCommand()
                ))
                .addMappedPoint(-52, 12, 180, 3)
                .setPower(0.75)
                .addMappedPoint(12, 12, 180, 6)
                .setPower(0.5)
                .addMappedPoint(35, 16, 215,2);
    }
}
