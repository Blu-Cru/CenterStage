package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.CenterCycleAudienceConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.CenterCycleBackdropConfig;

public abstract class AutoConfig {
    public static int position = 0;

    public abstract void build();
    public abstract void start();
    public abstract void run();

    public static AutoConfig config() {
        if(Globals.autoType == AutoType.CENTER_CYCLE && Globals.side == Side.AUDIENCE)
            return new CenterCycleAudienceConfig();
        return new CenterCycleBackdropConfig();
    }
}
