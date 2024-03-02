package org.firstinspires.ftc.teamcode.blucru.common.states;

// enum for type of auto to run
public enum AutoType {
    PRELOAD,
    CENTER_CYCLE,
    PERIMETER_CYCLE;

    // cycle through the auto types
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
