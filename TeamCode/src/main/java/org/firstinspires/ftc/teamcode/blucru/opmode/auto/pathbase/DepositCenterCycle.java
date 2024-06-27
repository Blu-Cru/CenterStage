package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretTurnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class DepositCenterCycle extends PIDPathBuilder {
    public DepositCenterCycle(double pixelHeight, double turretAngle) {
        super();
        this.setPower(0.32)
                .addMappedPoint(30, 16, 210, 4)
                .schedule(new SequentialCommandGroup(
                        new LockCommand(),
                        new OuttakeExtendCommand(pixelHeight),
                        new WaitCommand(500),
                        new TurretTurnCommand(270 + turretAngle * Globals.reflect)
                ))
                .addMappedPoint(50.3, 24, 200, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new LockReleaseCommand(1),
                        new WaitCommand(150),
                        new LockReleaseCommand(2)
                ))
                .waitMillis(700);
    }

    public DepositCenterCycle() {
        this(2.5, -10);
    }
}
