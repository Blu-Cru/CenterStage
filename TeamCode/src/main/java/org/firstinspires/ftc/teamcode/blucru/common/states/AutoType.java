package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum AutoType {
    PRELOAD(0),
    CENTER_CYCLE(1),
    PERIMETER_CYCLE(2),
    PARK(3);

    int value;
    AutoType(int value) {
        this.value = value;
    }

    public AutoType cycle() {
        value = (value + 1) % 4;
        return this;
    }
}
