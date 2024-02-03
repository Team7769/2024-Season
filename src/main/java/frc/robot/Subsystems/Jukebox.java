package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
    
public class Jukebox {

    private static Jukebox _instance;
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;

    private Timer _timer;

    private double manualElevatorSpeed;

    private ElevatorFeedforward _feedForward;

    private SparkPIDController _elevatorController;

    private TrapezoidProfile.Constraints _constraints;
    private TrapezoidProfile.State _goal;
    private TrapezoidProfile.State _profileSetpoint;

    private static double _oldPosition;
    

    public Jukebox()
    {
        // elevator motor setup
        // right elevator motor setup
        _elevatorR = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorR.setIdleMode(IdleMode.kBrake);
        _elevatorR.setSmartCurrentLimit(20, 100);
        _elevatorR.setInverted(true);
        _elevatorR.burnFlash();
        // left elevator motor setup
        _elevatorL = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorL.setIdleMode(IdleMode.kBrake);
        _elevatorL.setSmartCurrentLimit(20, 100);
        _elevatorL.setInverted(false);
        _elevatorL.burnFlash();
        // makes the right motor follow the left motor
        _elevatorR.follow(_elevatorL);

        _timer = new Timer();

        _feedForward = new ElevatorFeedforward(Constants.kElavatorFeedforwardKs,
        Constants.kElavatorFeedforwardKg, Constants.kElavatorFeedforwardKv);

        _constraints = new TrapezoidProfile.Constraints(Constants.kMaxVel, Constants.kMaxAccel);

        _goal = new TrapezoidProfile.State();
        _profileSetpoint = new TrapezoidProfile.State();

        _oldPosition = 0;
        manualElevatorSpeed = 0.0;
    }

    public static Jukebox getInstance()
    {
        if (_instance == null)
        {
            _instance = new Jukebox();
        }

        return _instance;
    }

    public void handleElevatorPosition() {
       
        var profile = new TrapezoidProfile(_constraints);
        _profileSetpoint = profile.calculate(_timer.get(), _profileSetpoint, _goal);
        _elevatorController.setReference(_profileSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0,
        _feedForward.calculate(_profileSetpoint.velocity));
    }

    public void setPosition(double position)
    {
        _goal = new TrapezoidProfile.State(position, 0);
        if (_oldPosition != position)
        {
            _timer.reset();
            _oldPosition = position;
        }
    }

<<<<<<< HEAD
    public void down(){}
=======
    private void setManualElevatorDown(){}
>>>>>>> a1b4e242a7b2753c27c45e6eef2e77e9335b97e7

    public void up(){}

    private void setSetpoint(double position)
    {
        _goal = new TrapezoidProfile.State(position, 0);

        switch (currentState) {
            case RESET:
                break;
            case HOLD:
                break;
            case DUMPAMP:
                break;
            case SETUPFORAMP:
                break;
            default:
                break;
        }
    }

    public boolean isItAtSetpoint()
    {
        return false;
    }

    public void holdPosition()
    {
        handleElevatorPosition();
        _elevatorL.set(Constants.speedToHoldElevator);
    }

    public void setManualElevatorSpeed(double s)
    {
        if (Math.abs(s) <= .10)
        {
            s = 0.0;
        }
        manualElevatorSpeed = s;
    }
    
    // @Override
<<<<<<< HEAD
    // public void logTelemetry(){}    


=======
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

    
>>>>>>> a1b4e242a7b2753c27c45e6eef2e77e9335b97e7
}
