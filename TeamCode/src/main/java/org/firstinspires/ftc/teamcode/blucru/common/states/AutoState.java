package org.firstinspires.ftc.teamcode.blucru.common.states;

// enum for the auto state
public enum AutoState {
    INIT, // initial state
    BUILD, // building trajectories state
    DETECTION, // detection state
    RUNNING, // running state
    PLACING_PURPLE,
    INTAKING,
    INTAKING_FAILED,
    DRIVING_THROUGH_TRUSS_TO_BACKDROP,
    DRIVING_THROUGH_TRUSS_TO_INTAKE,
    DEPOSITING,
    DEPOSITING_FAILED,
    STOP // stopped state
}
