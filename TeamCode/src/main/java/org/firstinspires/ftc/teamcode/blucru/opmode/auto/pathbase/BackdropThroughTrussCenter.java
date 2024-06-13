package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.systemcommand.OuttakeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropThroughTrussCenter extends PIDPathBuilder {
    public BackdropThroughTrussCenter () {
        super();
        this.setPower(0.8)
                .addMappedPoint(35, 12, 180, 6)
                .addMappedPoint(-33, 12, 180, 6);
    }
}
