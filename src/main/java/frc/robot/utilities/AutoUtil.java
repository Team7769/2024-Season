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
            case(MID_B_5_4):
                return new midB54();
            case(MID_B_4_5):
                return new midB34();
            case(FIVE_NOTE):
                return new FiveNote();  
            case (BOTTOM_NOTE):
                return new bottomAutoMode();          
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
        _autoChooser.addOption("Mid B 5 4 (4)", MID_B_5_4);
        _autoChooser.addOption("Mid B 3 4 (4)", MID_B_4_5);
        _autoChooser.addOption("Five Note", FIVE_NOTE);
        _autoChooser.addOption("Bottom Note", BOTTOM_NOTE);
    }

    // list of all auto modes 
    // every mode needs its own number 
    public static final int DO_NOTHING = 0;
    public static final int TEST_AUTONOMOUS = 1;
    public static final int MID_B_2_3 = 2;
    public static final int MID_B_5_4 = 3;
    public static final int MID_B_4_5 = 4;
    public static final int FIVE_NOTE = 5;
    public static final int BOTTOM_NOTE = 10;
}
