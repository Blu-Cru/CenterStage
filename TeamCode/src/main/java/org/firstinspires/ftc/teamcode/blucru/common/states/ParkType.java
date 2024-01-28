package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum ParkType {
    NONE,
    PERIMETER,
    CENTER;

    public ParkType cycle() {
        if(this == NONE) {
            return PERIMETER;
        } else if(this == PERIMETER) {
            return CENTER;
        } else {
            return NONE;
        }
    }
}
