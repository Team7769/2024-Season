package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
    
public class Jukebox {

    private static Jukebox _instance;
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;
    private ElevatorState currentState = ElevatorState.IDK;

    private final double k_Proportional=0;
    private final double k_integral=0;
    private final double k_derivative=0;


    

    public Jukebox()
    {
        _elevatorL = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR.follow(_elevatorL);
        _elevatorL.setIdleMode(IdleMode.kBrake);

        currentState = ElevatorState.IDK;
        
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

    private void handleCurrentState()
    {
        switch (currentState) {
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
            default:
                break;
        }
    }

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
