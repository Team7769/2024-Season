package frc.robot.Autonomous;

public class TOP_A_1_2_3_v2 extends EasyAuton {
    private static final AutoCmds[] auton = {
        AutoCmds.SCORE,
        AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        AutoCmds.SCORE,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        AutoCmds.SCORE,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        AutoCmds.SCORE,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        AutoCmds.SCORE
    };

    public TOP_A_1_2_3_v2() {
        super("TOP_A_1_2_3", auton);
    }
}
