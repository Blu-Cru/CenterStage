package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.intake.DropdownFullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.AutoReleasePurpleIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;

public class BackdropClosePreload extends PIDPathBuilder {
    public BackdropClosePreload() {
        super();
        this.setPower(0.8)
                .addMappedPoint(22, 55, 120, 6)
                .schedule(new DropdownFullRetractCommand())
                .addMappedPoint(33, 45, 135,4)
                .addMappedPoint(33.5, 37, 190)
                .schedule(
                        new SequentialCommandGroup(
                                new AutoReleasePurpleIntakeCommand(),
                                new WaitCommand(200),
                                new OuttakeExtendCommand(-1),
                                new TurretGlobalYCommand(42)
                        )
                )
                .waitMillis(200)
                .setPower(0.35)
                .addMappedPoint(Field.DEPOSIT_X, 40, 180, 2.5)
                .schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new LockReleaseCommand(2),
                                new WaitCommand(300),
                                new OuttakeRetractCommand(2)
                        )
                )
                .waitMillis(600);

    }
}
