package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropToStackPerimeter extends PIDPathBuilder {
    public BackdropToStackPerimeter() {
        super();
        this.setPower(0.5)
                .addMappedPoint(38, 60, 200, 5)
                .setPower(0.6)
                .addMappedPoint(-28, 60, 180, 10);
    }
}
