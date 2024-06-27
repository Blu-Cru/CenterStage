package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropToStackCenterAfterPreload extends PIDPathBuilder {
    public BackdropToStackCenterAfterPreload () {
        super();
        this.setPower(0.5)
                .addMappedPoint(38, 12, 180, 6)
                .setPower(0.6)
                .addMappedPoint(-33, 12, 180, 6);
    }
}
