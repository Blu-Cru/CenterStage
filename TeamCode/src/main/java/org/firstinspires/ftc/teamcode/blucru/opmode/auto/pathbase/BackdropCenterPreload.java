package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.AutoReleasePurpleIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropCenterPreload extends PIDPathBuilder {
    public BackdropCenterPreload() {
        super();
        this.setPower(0.8)
                .addMappedPoint(24, 48, 120, 6)
                .addMappedPoint(28, 30,190)

                .schedule(
                        new SequentialCommandGroup(
                                new AutoReleasePurpleIntakeCommand(),
                                new WaitCommand(250),
                                new OuttakeExtendCommand(-1),
                                new TurretGlobalYCommand(36)
                        )
                )

                .waitMillis(200)
                .setPower(0.35)
                .addMappedPoint(48.5, 36, 180, 2.5)
                .schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new LockReleaseCommand(2),
                                new WaitCommand(700),
                                new OuttakeRetractCommand(2)
                        )
                )
                .waitMillis(600);
    }
}
