package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.CenterCycleBackdropConfig;

public abstract class AutoConfig {
    public static int position = 0;

    public abstract void build();
    public abstract void start();
    public abstract void run();

    public AutoConfig config() {
        return new CenterCycleBackdropConfig();
    }
}
