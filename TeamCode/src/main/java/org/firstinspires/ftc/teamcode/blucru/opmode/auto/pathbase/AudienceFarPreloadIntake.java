package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class AudienceFarPreloadIntake extends PIDPathBuilder {
    public AudienceFarPreloadIntake() {
        super();
        this.setPower(0.7)
                .addMappedPoint(44, 53, 90, 6)
                .addMappedPoint(58, 36, 130, 3)
                .addMappedPoint(58, 30, 220);
    }
}
