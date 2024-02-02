package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.Constants;
    
public class Jukebox {
    private static Jukebox _instance;
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;

    enum ElevatorState
    {
        IDK,
        RAMPUP,
        SHOOT,
        RESET,
        DUMPAMP,
        SETUPFORAMP,
        EXTEND,
        CLIMB
    }

    

    public Jukebox()
    {
        _elevatorL = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
    }

    public static Jukebox getInstance()
    {
        if (_instance == null)
        {
            _instance = new Jukebox();
        }

        return _instance;
    }

    // @Override
    // public void logTelemetry(){}

    
}
