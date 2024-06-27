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
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class BackdropFarPreload extends PIDPathBuilder {
    public BackdropFarPreload () {
        super();
        this.setPower(0.7)
                .addPoint(Globals.mapPose(14, 45, 120), 8, false)
                .schedule(new DropdownFullRetractCommand())
                .addPoint(Globals.mapPose(10.5, 38, 190))
                .setPower(0.35)
                .schedule(
                        new SequentialCommandGroup(
                                new AutoReleasePurpleIntakeCommand(),
                                new WaitCommand(500),
                                new OuttakeExtendCommand(-1),
                                new TurretGlobalYCommand(30)
                        )
                )
                .waitMillis(300)
                .addPoint(Globals.mapPose(Field.DEPOSIT_X, 32, 180), 2.5)
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
