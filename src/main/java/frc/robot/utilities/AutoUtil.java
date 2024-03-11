package frc.robot.Utilities;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Autonomous.*;

public class AutoUtil {
    /**
     * Method that will return an autnomous mode depending on the one selected
     * 
     * @param _selectedAutoInit Determined by what is selected on the driver station
     * @return returns a specific autonomous mode that can be intialized and executed.
     */  
    public static AutonomousMode selectedAuto(int _selectedAutoInit)
    {
        switch (_selectedAutoInit)
        {
            case(DO_NOTHING):
                return new DoNothing();
            case(MID_B_3_4):
                return new Mid_B_3_4();
            case(MID_B_4_5):
                return new Mid_B_4_5();
            case(MID_B_3_4_5):
                return new Mid_B_3_4_5();
            case(MID_C_B_A_1):
                return new Mid_C_B_A_1();
            case(MID_B_A_1_2):
                return new Mid_B_A_1_2();
            case (TOP_A_1_2_3):
                return new Top_A_1_2_3();
            case(BOTTOM_5_4):
                return new Bottom_5_4();
        }
        
        return new DoNothing();
    }
    /**
     * This method is here to clear up the robot.java file 
     * It just adds the autonmous modes to the shuffle board to be chosen
     * 
     * @param _autoChooser takes the _autoChoose we use and adds all the autonomous modes
     */
    public static void autonmousDropDown(SendableChooser<Integer> _autoChooser)
    {
        _autoChooser.setDefaultOption("Do Nothing", DO_NOTHING);
        _autoChooser.addOption("Mid - C B A 1 (5)", MID_C_B_A_1);
        _autoChooser.addOption("Mid - B 3 4 (4)", MID_B_3_4);
        _autoChooser.addOption("Mid - B A 1 2 (5)", MID_B_A_1_2);
        _autoChooser.addOption("Top - A 1 2 3 (4)", TOP_A_1_2_3);
        _autoChooser.addOption("Bottom - 5 4 (3)", BOTTOM_5_4);
        
        // Disabled for now
        //_autoChooser.addOption("Mid - B 4 5 (4)", MID_B_4_5);
        //_autoChooser.addOption("Top 1 2 3", TOP_1_2_3);
        //_autoChooser.addOption("TestAutnomous", TEST_AUTONOMOUS);
        //_autoChooser.addOption("Mid B 2 3 (4)", MID_B_2_3);
        //_autoChooser.addOption("Mid B 3 4 5 (5)", MID_B_3_4_5);
    }

    // list of all auto modes 
    // every mode needs its own number 
    public static final int DO_NOTHING = 0;
    public static final int TEST_AUTONOMOUS = 1;
    public static final int MID_B_2_3 = 2;
    public static final int MID_B_3_4 = 3;
    public static final int MID_B_4_5 = 4;
    public static final int MID_B_3_4_5 = 5;
    public static final int MID_C_B_A_1 = 6;
    public static final int MID_B_A_1_2 = 7;
    public static final int TOP_A_1_2_3 = 8;
    public static final int TOP_1_2_3 = 9;
    public static final int BOTTOM_5_4 = 10;    
}
