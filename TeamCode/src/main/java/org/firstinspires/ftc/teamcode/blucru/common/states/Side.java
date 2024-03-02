package org.firstinspires.ftc.teamcode.blucru.common.states;

// starting side for the robot in auto
public enum Side {
    CLOSE,
    FAR;


    // flip the starting side
    public Side flip() {
        if(this == CLOSE) {
            return FAR;
        } else {
            return CLOSE;
        }
    }
}
