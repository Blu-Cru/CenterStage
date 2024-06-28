package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.LockReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.subsystemcommand.outtake.TurretGlobalYCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Field;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;

public class AudienceCenterPreloadDeposit extends PIDPathBuilder {
    public static AudienceCenterPreloadDeposit get(AutoType autoType) {
        if(autoType == AutoType.CENTER_CYCLE) return new AudienceCenterPreloadDeposit(30, 36);
        else return new AudienceCenterPreloadDeposit(42, 36);
    }

    private AudienceCenterPreloadDeposit(double globalYWhite, double globalYYellow) {
        super();
        this.setPower(0.35)
                .schedule(new SequentialCommandGroup(
                        new OuttakeExtendCommand(-1),
                        new TurretGlobalYCommand(globalYWhite)
                ))
                .addMappedPoint(Field.DEPOSIT_X, 36, 180, 2.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(100),
                        new LockReleaseCommand(1),
                        new WaitCommand(200),
                        new OuttakeExtendCommand(1),
                        new WaitCommand(100),
                        new TurretGlobalYCommand(globalYYellow),
                        new WaitCommand(100),
                        new OuttakeExtendCommand(-1),
                        new WaitCommand(200),
                        new LockReleaseCommand(2)
                ))
                .waitMillis(1000);
    }

    public static AudienceCenterPreloadDeposit get() {
        return get(Globals.autoType);
    }
}
