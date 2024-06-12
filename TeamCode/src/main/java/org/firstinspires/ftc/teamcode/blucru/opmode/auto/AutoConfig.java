package org.firstinspires.ftc.teamcode.blucru.opmode.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.states.AutoType;
import org.firstinspires.ftc.teamcode.blucru.common.states.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.states.Randomization;
import org.firstinspires.ftc.teamcode.blucru.common.states.Side;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.CenterCycleAudienceConfig;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.config.CenterCycleBackdropConfig;

public abstract class AutoConfig {
    public static int position = 0;

    public abstract void build();
    public abstract void start(Randomization randomization);
    public abstract void run();
    public abstract void telemetry(Telemetry telemetry);

    public static AutoConfig config() {
        if(Globals.side == Side.AUDIENCE) {
            if(Globals.autoType == AutoType.CENTER_CYCLE)
                return new CenterCycleAudienceConfig();
        }
        else if(Globals.side == Side.BACKDROP) {
            if(Globals.autoType == AutoType.CENTER_CYCLE)
                return new CenterCycleBackdropConfig();
        }
        return null;
    }
}
