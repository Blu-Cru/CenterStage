package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.transition;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class BackdropToStackPerimeter extends PIDPathBuilder {
    public BackdropToStackPerimeter() {
        super();
        this.setPower(0.5)
                .addMappedPoint(38, 60, 160, 5)
                .setPower(0.6)
                .addMappedPoint(-35, 60, 180, 10);
    }
}
