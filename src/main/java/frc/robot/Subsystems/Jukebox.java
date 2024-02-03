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
    private static double _oldPosition;
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;
    private CANSparkMax _feeder;
    private CANSparkMax _shooterAngle;
    private SparkPIDController _elevatorController;
    private ElevatorFeedforward _feedForward;
    private Notebox noteboxCurrentState = Notebox.IDK;
    private TrapezoidProfile.Constraints _constraints;
    private TrapezoidProfile.State _goal;
    private TrapezoidProfile.State _profileSetpoint;
    private Timer _timer;

    private double manualElevatorSpeed;
    

    public Jukebox()
    {
        // motor setup
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

        // shooter angle motor setup
        _shooterAngle = new CANSparkMax(Constants.kShooterAngleId, MotorType.kBrushless);
        _shooterAngle.setIdleMode(IdleMode.kBrake);
        _shooterAngle.setSmartCurrentLimit(20, 100);
        _shooterAngle.setInverted(false);
        _shooterAngle.burnFlash();

        // feeder motor setup
        _feeder = new CANSparkMax(Constants.kFeederId, MotorType.kBrushless);
        _feeder.setIdleMode(IdleMode.kBrake);
        _feeder.setSmartCurrentLimit(20, 100);
        _feeder.setInverted(false);
        _feeder.burnFlash();

        // the timer is needed for handleElevatorPosistion
        _timer = new Timer();

        // creates the feed foward for the elevator
        _feedForward = new ElevatorFeedforward(Constants.kElavatorFeedforwardKs,
        Constants.kElavatorFeedforwardKg, Constants.kElavatorFeedforwardKv);

        // the constraints our elevator has
        _constraints = new TrapezoidProfile.Constraints(Constants.kMaxVel, Constants.kMaxAccel);

        // our desired state and current state
        _goal = new TrapezoidProfile.State();
        _profileSetpoint = new TrapezoidProfile.State();

        // used to track the old position of the elevator only used to see if we actually move
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
        _elevatorController.setReference(_profileSetpoint.position, com.revrobotics.CANSparkBase.ControlType.kPosition, 0,
        _feedForward.calculate(_profileSetpoint.velocity));
    }
    /**
     * Method that will set the angle of the shooter
     */
    public void setShooterAngle(double desiredAngle) {

    }
    
    /**
     * Sets the elevator to where it needs to be and if the position changes we reset the timer and update the old position to the new position
     * @param position takes a double and makes the goal state.
     */
    public void setElevatorPosition(double position)
    {
        _goal = new TrapezoidProfile.State(position, 0);
        if (_oldPosition != position)
        {
            _timer.reset();
            _oldPosition = position;
        }
    }

    public void setManualElevatorDown(){}

    public void up(){}

    public void setSetpoint(double position)
    {
        
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
    // public void logTelemetry(){}

    // private void IDK(){}
    // private void RESET(){}
    // private void HOLD_POSITION(){}
    // private void UP_ELEVATOR(){}
    // private void DOWN_ELEVATOR(){}

    public void handleCurrentState()
    {
        switch (noteboxCurrentState) {
            case IDK:
                break;
            case RESET:
                break;
            case HOLD_POSITION:
                break;
            case UP_ELEVATOR:
                break;
            case DOWN_ELEVATOR:
                break;
            default:
                break;
        }
    }

    public void setState(Notebox n)
    {
        switch (n) {
            case IDK:
                break;
            case RESET:
                break;
            case HOLD_POSITION:
                break;
            case UP_ELEVATOR:
                break;
            case DOWN_ELEVATOR:
                break;
            default:
                break;
        }

        noteboxCurrentState = n;
    }

    public boolean isFinish(Notebox c)
    {
        if(noteboxCurrentState != c){
            return true;
        } else {
            return false;
        }
    }   
}