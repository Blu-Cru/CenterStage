package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class AudienceCenterToBackdropPreload extends PIDPathBuilder {
    public AudienceCenterToBackdropPreload() {
        super();
        this.setPower(0.5)
                .addMappedPoint(-52, 12, 180, 3)
                .setPower(0.7)
                .addMappedPoint(-12, 12, 180, 6)
                .setPower(0.5)
                .addMappedPoint(30, 12, 220,2);
    }
}
