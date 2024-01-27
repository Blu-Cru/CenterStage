package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum ParkType {
    NONE(0),
    PERIMETER(1),
    CENTER(2);

    int value;
    ParkType(int value) {
        this.value = value;
    }

    public ParkType cycle() {
        value = (value + 1) % 3;
        return this;
    }
}
