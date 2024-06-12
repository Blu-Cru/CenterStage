package org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class AudienceFarPreloadIntake extends PIDPathBuilder {
    public AudienceFarPreloadIntake() {
        super();
        this.setPower(0.8)
                .addMappedPoint(-38, 45, 120,6)
                .addMappedPoint(-33, 36, 160, false)
                .waitMillis(200)
                .addMappedPoint(-44, 30, 180, 6)
                .addMappedPoint(-58, 24, 180)
                .waitMillis(1000)
                .addMappedPoint(-45, 24,180)
                .build();
    }
}
