package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
    
public class Jukebox {
    

    

    public Jukebox()
    {
        _elevatorL = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR.follow(_elevatorL);
        _elevatorL.setIdleMode(IdleMode.kBrake);
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

    private void switchCurrentState(){}

    public void setState(ElevatorState e)
    {
        switch (e) {
            case IDK:
                
                break;
        
            case RAMPUP:
                break;
            case SHOOT:
                break;
            case RESET:
                break;
            case DUMPAMP:
                break;
            case SETUPFORAMP:
                break;
            case EXTEND:
                break;
        }
    }

    
}
