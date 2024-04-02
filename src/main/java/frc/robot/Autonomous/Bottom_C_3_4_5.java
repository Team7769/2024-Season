package frc.robot.Autonomous;

public class Bottom_C_3_4_5 extends EasyAuton {
    private static final AutoCmds[] auton = {
        AutoCmds.SHOOT_RUSHED,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_AIM,
        AutoCmds.SHOOT_RUSHED,
        AutoCmds.FOLLOW,
        AutoCmds.FOLLOW_WITH_FULL_AIM,
        AutoCmds.SHOOT_RUSHED
    };

    public Bottom_C_3_4_5() {
        super("Bottom-C-3-4-5", auton);
    }
}
