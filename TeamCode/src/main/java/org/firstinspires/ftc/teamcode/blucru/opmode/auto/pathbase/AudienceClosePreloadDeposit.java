package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.OuttakeIncrementCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class AudienceClosePreloadDeposit extends PIDPathBuilder {
    public static AudienceClosePreloadDeposit get(AutoType autoType) {
        if(autoType == AutoType.CENTER_CYCLE) return new AudienceClosePreloadDeposit(34, 41.5);
        else return new AudienceClosePreloadDeposit(34, 41.5);
    }

    private AudienceClosePreloadDeposit(double globalYWhite, double globalYYellow) {
        super();
        this.setPower(0.35)
                .schedule(new SequentialCommandGroup(
                        new OuttakeExtendCommand(0),
                        new TurretGlobalYCommand(globalYWhite)
                ))
                .addMappedPoint(Field.DEPOSIT_X, 40, 180, 2.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(100),
                        new LockReleaseCommand(1),
                        new WaitCommand(120),
                        new OuttakeIncrementCommand(1),
                        new TurretGlobalYCommand(globalYYellow),
                        new WaitCommand(100),
                        new OuttakeExtendCommand(-0.2),
                        new WaitCommand(200),
                        new LockReleaseCommand(2),
                        new WaitCommand(200),
                        new OuttakeRetractCommand(2)
                ))
                .waitMillis(1500);
    }

    public static AudienceClosePreloadDeposit get() {
        return get(Globals.autoType);
    }
}
