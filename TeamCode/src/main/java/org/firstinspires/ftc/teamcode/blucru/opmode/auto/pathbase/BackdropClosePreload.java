package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.DropdownPartialRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.IntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropClosePreload extends PIDPathBuilder {
    public BackdropClosePreload() {
        super();
        this.setPower(0.8)
                .addMappedPoint(22, 55, 120, 6)
                .addMappedPoint(33, 45, 135,4)
                .schedule(
                        new SequentialCommandGroup(
                                new DropdownPartialRetractCommand(),
                                new WaitCommand(300),
                                new IntakePowerCommand(-0.45),
                                new WaitCommand(1000),
                                new StopIntakeCommand()
                        )
                )
                .addMappedPoint(30.5, 37, 190)
                .schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new OuttakeExtendCommand(-1),
                                new TurretGlobalYCommand(42)
                        )
                )
                .waitMillis(200)
                .setPower(0.35)
                .addMappedPoint(48.5, 40, 180, 2.5)
                .schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new LockReleaseCommand(2),
                                new WaitCommand(300),
                                new OuttakeRetractCommand(2)
                        )
                )
                .setPower(0.7)
                .waitMillis(500)
                .addMappedPoint(40, 12, 180);

    }
}
