package frc.robot.Subsystems;

    public enum JukeboxEnum
    {
        IDK, // everything is idk
        ELEVATOR_UP, // set the elevator to a positive voltage
        TILT_UP, // this tilt up with the shooter
        RAMP_UP_SHOOTER, // ramp up the motors
        RAMP_DOWN_SHOOTER, // set the motors to 0
        TILT_DOWN, // this tilt down with the shooter
        ELEVATOR_DOWN, // set the elevator down
        HOLD_POSITION, // check to make sure some states are finish before another state start
        SPIT, // Spits the note out of the jukebox
        FEED // feeds the note to the shooter wheels

    }