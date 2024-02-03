package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
    
public class Jukebox {

    private static Jukebox _instance;
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;
    private ElevatorState currentState = ElevatorState.IDK;

    private final double kP = 0.015;
    private final double kI = 0.0;
    private final double kD = 0.001;
    private final double kFF = 0.0;
    private final double kIz = 0.0;
    private final double kMaxOutput = 1.00;
    private final double kMinOutput = -1.00;
    private final double kMaxVel = 5;
    private final double kMaxAccel = 5;
    private final double kAllowedError = 3;

    private ElevatorFeedforward _feedForward;

    private final TrapezoidProfile.Constraints _constraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel);
    private TrapezoidProfile.State _goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State _profileSetpoint = new TrapezoidProfile.State();
    private final double k_Proportional=0;
    private final double k_integral=0;
    private final double k_derivative=0;


    

    public Jukebox()
    {
        _elevatorR = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR.setIdleMode(IdleMode.kBrake);
        _elevatorR.setSmartCurrentLimit(20, 100);
        _elevatorR.setInverted(true);
        _elevatorR.burnFlash();

        _elevatorL = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorL.setIdleMode(IdleMode.kBrake);
        _elevatorL.setSmartCurrentLimit(20, 100);
        _elevatorL.setInverted(false);
        _elevatorL.burnFlash();
        
        _elevatorR.follow(_elevatorL);

        currentState = ElevatorState.IDK;
        
        _feedForward = new ElevatorFeedforward(Constants.kElavatorFeedforwardKs,
        Constants.kElavatorFeedforwardKg, Constants.kElavatorFeedforwardKv);
    }

    public static Jukebox getInstance()
    {
        if (_instance == null)
        {
            _instance = new Jukebox();
        }

        return _instance;
    }

    private void handleElevatorPosition() {
       
        var profile = new TrapezoidProfile(_constraints, _goal, _profileSetpoint);
        _profileSetpoint = profile.calculate(0.02);
        _elevatorL.setReference(_profileSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0,
        _feedForward.calculate(_profileSetpoint.velocity));
    }

    private void setSetpoint(double position)
    {
        _goal = new TrapezoidProfile.State(position, 0);
    }
    
    // @Override
    // public void logTelemetry(){}


    private void IDK(){
        _elevatorL.set(0);
        _elevatorR.set(0);
    }
    private void RAMPUP(){}
    private void SHOOT(){}
    private void RESET(){}
    private void DUMPAMP(){}
    private void SETUPFORAMP(){}
    private void EXTEND(){}
    private void CLIMB(){}

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
            case CLIMB:
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
            case CLIMB:
                break;
        }
    }

    
}
