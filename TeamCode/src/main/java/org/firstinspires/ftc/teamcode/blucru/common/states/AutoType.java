package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum AutoType {
    PRELOAD,
    CENTER_CYCLE,
    PERIMETER_CYCLE;

    public AutoType cycle() {
        if(this == PRELOAD) {
            return CENTER_CYCLE;
        } else if(this == CENTER_CYCLE) {
            return PERIMETER_CYCLE;
        } else {
            return PRELOAD;
        }
    }
}
