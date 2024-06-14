package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.TurretTurnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class DepositCenterCycle extends PIDPathBuilder {
    public DepositCenterCycle(double pixelHeight, double turretAngle) {
        super();
        this.setPower(0.35)
                .addMappedPoint(35, 12, 205, 4)
                .schedule(new SequentialCommandGroup(
                        new OuttakeExtendCommand(pixelHeight),
                        new WaitCommand(500),
                        new TurretTurnCommand(270 + turretAngle * Globals.reflect)
                    )
                )
                .addMappedPoint(48, 20, 205, 2.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new LockReleaseCommand(1),
                        new WaitCommand(200),
                        new LockReleaseCommand(2)
                    )
                )
                .waitMillis(1000);
    }

    public DepositCenterCycle() {
        this(2.5, 0);
    }
}
