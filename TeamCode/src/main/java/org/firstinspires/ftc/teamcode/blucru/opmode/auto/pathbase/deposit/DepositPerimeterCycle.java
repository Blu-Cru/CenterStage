package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.deposit;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretTurnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class DepositPerimeterCycle extends PIDPathBuilder {
    public DepositPerimeterCycle() {
        this(1.8, 4);
    }

    public DepositPerimeterCycle(double pixelHeight, double turretAngle) {
        super();
        this.setPower(0.32)
                .addMappedPoint(30, 56, 150, 4)
                .schedule(new SequentialCommandGroup(
                        new LockCommand(),
                        new OuttakeExtendCommand(pixelHeight),
                        new WaitCommand(500),
                        new TurretTurnCommand(270 + turretAngle * Globals.reflect)
                ))
                .addMappedPoint(50.7, 47, 165, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(450),
                        new LockReleaseCommand(1),
                        new WaitCommand(150),
                        new LockReleaseCommand(2)
                ))
                .waitMillis(700);
    }
}
