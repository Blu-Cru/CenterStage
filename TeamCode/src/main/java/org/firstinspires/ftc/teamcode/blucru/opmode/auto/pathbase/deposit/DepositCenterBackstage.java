package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.deposit;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.OuttakeWristBackstageCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class DepositCenterBackstage extends PIDPathBuilder {
    public DepositCenterBackstage() {
        super();
        this.setPower(0.7)
                .schedule(new SequentialCommandGroup(
                        new LockCommand(),
                        new InstantCommand(
                                () -> Robot.getInstance().outtake.setTargetPixelHeight(-1)
                        ),
                        new WaitCommand(10),
                        new OuttakeWristBackstageCommand()
                ))
                .addMappedPoint(42, 12, 180, 6)
                .schedule(new SequentialCommandGroup(
                        new LockReleaseCommand(2),
                        new OuttakeRetractCommand()
                ))
                .waitMillis(400);
    }
}
