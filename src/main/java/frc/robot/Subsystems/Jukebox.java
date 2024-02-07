package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Enums.JukeboxEnum;
    
public class Jukebox extends Subsystem{
    
    private static Jukebox _instance;
    private static double _oldPosition;

    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;
    private CANSparkMax _feeder;
    private CANSparkMax _shooterAngle;
    private CANSparkMax _shooterL;
    private CANSparkMax _shooterR;

    private SparkPIDController _shooterAngleController;
    private SparkPIDController _elevatorController;
    private SparkPIDController _shooterController;

    private ElevatorFeedforward _feedForward;
    private JukeboxEnum noteboxCurrentState = JukeboxEnum.IDLE;

    private TrapezoidProfile.Constraints _constraints;
    private TrapezoidProfile.State _goal;
    private TrapezoidProfile.State _profileSetpoint;
    private TrapezoidProfile.Constraints _shooterConstraints;
    private TrapezoidProfile.State _shooterGoal;
    private TrapezoidProfile.State _shooterSetPoint;

    private DigitalInput _noteHolder;
    private DigitalInput _noteShooter;

    private Timer _timer;

    // Create a constant speed for each of the motors 
    private double _shooterSpeed = 0;
    private double _elevatorSpeed = 0;
    private double _angleSpeed = 0;

    // private final double kElavatorFeedforwardKs = 0;
    // private final double kElavatorFeedforwardKv = 0;
    // private final double kElavatorFeedforwardKg = 0;

    // private final double kP = 0.015;
    // private final double kI = 0.0;
    // private final double kD = 0.001;
    // private final double kFF = 0.0;
    // private final double kIz = 0.0;
    // private final double kMaxOutput = 1.00;
    // private final double kMinOutput = -1.00;
    // private final double kMaxVel = 5;
    // private final double kMaxAccel = 5;
    // private final double kAllowedError = 3;
    // private final double speedToHoldElevator = 0.0;
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
        _elevatorL.burnFlash();

        // left shooter motor setup
        _shooterL = new CANSparkMax(Constants.kShooterLeftMotorId, MotorType.kBrushless);
        _shooterL.setIdleMode(IdleMode.kCoast);
        _shooterL.setSmartCurrentLimit(20, 100);
        _shooterL.setInverted(true);
        _shooterL.burnFlash();

        // right shooter motor setup
        _shooterR = new CANSparkMax(Constants.kShooterRightMotorId, MotorType.kBrushless);
        _shooterR.setIdleMode(IdleMode.kCoast);
        _shooterR.setSmartCurrentLimit(20, 100);
        _shooterR.setInverted(false);
        _shooterR.burnFlash();

        // make the left shooter moter follow the right
        _shooterR.follow(_shooterL);

        // makes the right motor follow the left motor
        _elevatorR.follow(_elevatorL);

        // shooter angle motor setup
        _shooterAngle = new CANSparkMax(Constants.kShooterAngleId, MotorType.kBrushless);
        _shooterAngle.setIdleMode(IdleMode.kBrake);
        _shooterAngle.setSmartCurrentLimit(20, 100);
        _shooterAngle.setInverted(false);
        _shooterAngle.burnFlash();
        _shooterAngleController = _shooterAngle.getPIDController();

        _shooterController = _shooterL.getPIDController();
        _shooterController.setOutputRange(0, 1);

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

        // our desired state and current state with the shooter
        _shooterGoal = new TrapezoidProfile.State();
        _shooterSetPoint = new TrapezoidProfile.State();

        // used to track the old position of the elevator only used to see if we actually move
        _oldPosition = 0;
        manualElevatorSpeed = 0.0;

        _noteHolder = new DigitalInput(1);
        _noteShooter = new DigitalInput(2);

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
        var profile = new TrapezoidProfile(_constraints);
        _profileSetpoint = profile.calculate(_timer.get(), _profileSetpoint, _goal);
        _elevatorController.setReference(_profileSetpoint.position, com.revrobotics.CANSparkBase.ControlType.kPosition, 0,
        _feedForward.calculate(_profileSetpoint.velocity));
    }

    /**
     * Method that will set the angle of the shooter.
     * This should also be apply to the tilt angle too.
     * Once the shooter angle is set it should auto apply the tilt angle.
     * 1 means it set it clockwise
     * -1 means it set it counterclockwise
     */
    private void setShooterAngle(double desiredPosition) {
        _shooterAngleController.setReference(desiredPosition, com.revrobotics.CANSparkBase.ControlType.kPosition, 0);
    }

    /**
     * ShooterDeploy ramps up the shooter motors to the max
     */
    private void setShooterSpeed(double v)
    {
        _shooterL.set(v);
    }
    
    /**
     * Sets the elevator to where it needs to be and if the position changes we reset the timer and update the old position to the new position
     * @param position takes a double and makes the goal state.
     */
    private void setElevatorPosition(double position)
    {
        if (_oldPosition != position)
        {
            _goal = new TrapezoidProfile.State(position, 0);
            _timer.reset();
            _oldPosition = position;
        }
    }

    private void feed(double v)
    {
        _feeder.set(v);
    }

    private void spit( double v)
    {
        _feeder.set(-v);
    }

    private void setManualElevatorSpeed(double s)
    {
        if (Math.abs(s) <= .10)
        {
            s = 0.0;
        }
        manualElevatorSpeed = s;
    }
    
    public void logTelemetry() {
        SmartDashboard.putNumber("Elevator motor left enconder position", _elevatorL.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor left enconder velocity", _elevatorL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator motor left speed", _elevatorL.get());

        SmartDashboard.putNumber("Elevator motor right enconder position", _elevatorR.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor right enconder velocity", _elevatorR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator motor right speed", _elevatorR.get());

        SmartDashboard.putNumber("Feeder motor enconder position", _feeder.getEncoder().getPosition());
        SmartDashboard.putNumber("Feeder motor enconder velocity", _feeder.getEncoder().getVelocity());
        SmartDashboard.putNumber("Feeder motor speed", _feeder.get());
        
        SmartDashboard.putNumber("Shooter angle motor enconder position", _shooterAngle.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter angle motor enconder velocity", _shooterAngle.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter angle motor speed", _shooterAngle.get());

        SmartDashboard.putBoolean("is the note pass the shooter limit switch", _noteHolder.get());
        SmartDashboard.putBoolean("is the note in the correct position in the holder", _noteShooter.get());
    }

    private void score() {

    }

    private void prepAmp() {

    }

    private void prepSpeaker() {

    }

    private void prepTrap() {

    }

    private void reset() {

    }

    private void extendForClimb() {

    }

    private void climb() {

    }

    private void idle() {
        setElevatorPosition(0);
        setShooterAngle(0);
        setShooterSpeed(0);
    }

    private void manual() {

    }

    public void handleCurrentState()
    {
        switch(noteboxCurrentState){
            case MANUAL:
                manual();
                break;
            case SCORE:
                score();
                break;
            case PREP_SPEAKER:
                prepSpeaker();
                break;
            case PREP_AMP:
                prepAmp();
                break;
            case PREP_TRAP:
                prepTrap();
                break;
            case RESET:
                reset();
                break;
            case EXTEND_FOR_CLIMB:
                extendForClimb();
                break;
            case CLIMB:
                climb();
                break;
            default:
               idle();
                break;
        }

        switch (noteboxCurrentState) {
            case MANUAL:
                break;
            default:
                handleElevatorPosition();
                break;
        }
    }

    public void setState(JukeboxEnum n)
    {
        noteboxCurrentState = n;
    }
}