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
                return new TestAutonomous();
            case(TEST_AUTONOMOUS):
                return new TestAutonomous();
            case(MID_B_2_3):
                return new midB23();
            case(MID_B_3_4):
                return new midB34();
            case(MID_B_4_5):
                return new midB45();
            case(MID_B_3_4_5):
                return new midB345();
            case(MID_C_B_3_4):
                return new midCB34();
            case(FIVE_NOTE):
                return new FiveNote();     
            case(TOP_1_2_3):
                return new TOP123();       
        }
        
        return null;
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
        _autoChooser.addOption("TestAutnomous", TEST_AUTONOMOUS);
        _autoChooser.addOption("Mid B 2 3 (4)", MID_B_2_3);
        _autoChooser.addOption("Mid B 3 4 (4)", MID_B_3_4);
        _autoChooser.addOption("Mid B 4 5 (4)", MID_B_4_5);
        _autoChooser.addOption("Mid B 3 4 5 (5)", MID_B_3_4_5);
        _autoChooser.addOption("Mid C B 3 4 (5)", MID_C_B_3_4);
        _autoChooser.addOption("Five Note", FIVE_NOTE);
        _autoChooser.addOption("Top 1 2 3", TOP_1_2_3);
    }

    // list of all auto modes 
    // every mode needs its own number 
    public static final int DO_NOTHING = 0;
    public static final int TEST_AUTONOMOUS = 1;
    public static final int MID_B_2_3 = 2;
    public static final int MID_B_3_4 = 3;
    public static final int MID_B_4_5 = 4;
    public static final int MID_B_3_4_5 = 5;
    public static final int MID_C_B_3_4 = 6;
    public static final int FIVE_NOTE = 7;
    public static final int TOP_1_2_3 = 8;
    
}
