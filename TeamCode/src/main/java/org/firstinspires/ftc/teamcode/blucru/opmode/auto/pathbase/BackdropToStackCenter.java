package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropToStackCenter extends PIDPathBuilder {
    public BackdropToStackCenter() {
        super();
        this.setPower(0.5)
                .addMappedPoint(38, 12, 200, 6)
                .setPower(0.6)
                .addMappedPoint(-33, 12, 180, 9);
    }
}
