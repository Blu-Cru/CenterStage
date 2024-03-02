package org.firstinspires.ftc.teamcode.blucru.common.states;

// enum for the alliance color
public enum Alliance {
    RED,
    BLUE;

    // flip the alliance color
    public Alliance flip() {
        if(this == RED) {
            return BLUE;
        } else {
            return RED;
        }
    }
}
