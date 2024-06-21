package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class CenterDepositFailsafe extends PIDPathBuilder {
    public CenterDepositFailsafe() {
        super();
        this.setPower(0.5)
                .addMappedPoint(40, 16, 225, 4);
    }
}
