package frc.robot.utilities;

import frc.robot.Autonomous.AutonomousMode;
import frc.robot.Autonomous.TestAutonomous;

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
        }
        return null;
    }

    // list of all auto modes
    public static final int DO_NOTHING = 0;
}
