package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.preload;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropToStackPerimeterAfterPreload extends PIDPathBuilder {
    public BackdropToStackPerimeterAfterPreload() {
        super();
        this.setPower(0.5)
                .addMappedPoint(38, 60, 180, 6)
                .setPower(0.6)
                .addMappedPoint(-33, 60, 180, 7);
    }
}
