package org.firstinspires.ftc.teamcode.blucru.common.states;

// enum for different parking locations
public enum ParkType {
    NONE,
    PERIMETER,
    CENTER;

    // cycle through the parking locations
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
