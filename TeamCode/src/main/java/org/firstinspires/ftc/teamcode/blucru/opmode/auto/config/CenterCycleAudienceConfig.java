package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.pathbase.AudienceFarPreloadIntake;

public class CenterCycleAudienceConfig extends AutoConfig {
    Path preloadIntakeFar, preloadIntakeCenter, preloadIntakeClose;
    public CenterCycleAudienceConfig() {

    }

    public void build() {
        preloadIntakeFar = new AudienceFarPreloadIntake().build();
    }

    public void start() {
        preloadIntakeFar.start();
    }

    public void run() {
        preloadIntakeFar.run();
    }
}
