package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class AudienceClosePreloadDeposit extends PIDPathBuilder {
    public AudienceClosePreloadDeposit() {
        super();
        this.setPower(0.4)
                .addMappedPoint(48.5, 40, 180, 2.5)
                .waitMillis(200);
    }
}
