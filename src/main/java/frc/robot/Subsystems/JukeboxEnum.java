package frc.robot.Subsystems;

    public enum JukeboxEnum
    {
        IDK, // everything is idk
        ELEVATOR_UP, // set the elevator to a positive voltage
        POLLER_UP, // Polls up the note to the holder
        TILT_UP, // this tilt up with the shooter
        IS_NOTE_IN_HOLDER, // Checks to make sure the note is in the holder, but does not touch the shooter motors
        RAMP_UP_SHOOTER, // ramp up the motors
        IS_NOTE_IN_SHOOTER_POSITION, // check to make sure the note is in a shooter position
        SHOOT_NOTE, // spin the poller up motor to the shooter
        TILT_DOWN, // this tilt down with the shooter
        ELEVATOR_DOWN, // set the elevator down
        IS_STATE_FINISH // check to make sure some states are finish before another state start


    }