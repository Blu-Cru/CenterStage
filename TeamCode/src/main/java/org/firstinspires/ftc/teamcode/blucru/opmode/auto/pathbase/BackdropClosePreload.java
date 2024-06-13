package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.AutoReleasePurpleIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropClosePreload extends PIDPathBuilder {
    public BackdropClosePreload() {
        super();
        this.setPower(0.8)
                .addMappedPoint(22, 55, 120, 6)
                .addMappedPoint(33, 45, 135,4)
                .schedule(
                        new AutoReleasePurpleIntakeCommand(300)

                )
                .addMappedPoint(33.5, 37, 190)
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
                                new WaitCommand(300),
                                new LockReleaseCommand(2),
                                new WaitCommand(700),
                                new OuttakeRetractCommand(2)
                        )
                )
                .waitMillis(600);

    }
}
