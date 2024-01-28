package org.firstinspires.ftc.teamcode.blucru.common.states;

public enum Alliance {
    RED,
    BLUE;

    public Alliance flip() {
        if(this == RED) {
            return BLUE;
        } else {
            return RED;
        }
    }
}
