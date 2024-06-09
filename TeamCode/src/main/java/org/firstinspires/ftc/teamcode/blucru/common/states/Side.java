package org.firstinspires.ftc.teamcode.blucru.common.states;

// starting side for the robot in auto
public enum Side {
    BACKDROP,
    AUDIENCE;


    // flip the starting side
    public Side flip() {
        if(this == BACKDROP) {
            return AUDIENCE;
        } else {
            return BACKDROP;
        }
    }
}
