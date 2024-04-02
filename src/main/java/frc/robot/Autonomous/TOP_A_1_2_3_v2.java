package frc.robot.Autonomous;

public class Top_A_1_2_3_v2 extends EasyAuton {
    private static final AutoCmds[] auton = {
        // AutoCmds.SCORE,
        // AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        // AutoCmds.SCORE,
        // AutoCmds.FOLLOW,
        // AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        // AutoCmds.SCORE,
        // AutoCmds.FOLLOW,
        // AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        // AutoCmds.SCORE,
        // AutoCmds.FOLLOW,
        // AutoCmds.FOLLOW_WITH_FULL_SPKLL_AIM,
        // AutoCmds.SCORE
        AutoCmds.JBX_AIM,
        AutoCmds.SHOOT,
        AutoCmds.FOLLOW_WITH_FULL_AIM,
        AutoCmds.SHOOT,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_AIM,
        AutoCmds.SHOOT,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_AIM,
        AutoCmds.SHOOT,
        AutoCmds.FOLLOW
    };

    public Top_A_1_2_3_v2() {
        super("Top A-1-2-3", auton);
    }
}
