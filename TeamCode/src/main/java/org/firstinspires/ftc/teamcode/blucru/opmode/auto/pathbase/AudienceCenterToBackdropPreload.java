package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownFullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class AudienceCenterToBackdropPreload extends PIDPathBuilder {
    public AudienceCenterToBackdropPreload() {
        super();
        this.setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new IntakePowerCommand(1),
                        new DropdownFullRetractCommand(),
                        new WaitCommand(100),
                        new LockCommand(),
                        new IntakeStopCommand()
                ))
                .addMappedPoint(-52, 12, 180, 3)
                .setPower(0.75)
                .addMappedPoint(12, 12, 180, 6)
                .setPower(0.5)
                .addMappedPoint(35, 16, 215,2);
    }
}
