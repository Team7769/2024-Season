package frc.robot.Enums;

    public enum JukeboxEnum
    {
        /**Idle state, most of the match. */
        IDLE,

        /**Score the note. */
        SCORE,

        /**Spins up the shooter for a speaker shot. */
        PREP_SPEAKER,

        /**Brings the jukebox to the amp scoring position. */
        PREP_AMP,

        /**Brings the jukebox to the trap scoring position. */
        PREP_TRAP,

        /**Resets the jukebox to the IDLE configuration. */
        RESET,

        /**Extends the elevator for climbing in the endgame. */
        EXTEND_FOR_CLIMB,

        /**Retracts the elevator to climb in the endgame. */
        CLIMB,

        /**Manual control */
        MANUAL
    }