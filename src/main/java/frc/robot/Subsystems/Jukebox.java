package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Utilities.OneDimensionalLookup;
    
public class Jukebox extends Subsystem{
    
    private static Jukebox _instance;

    // Motor Controllers
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;
    private CANSparkMax _feeder;
    private CANSparkMax _shooterAngle;
    private CANSparkMax _shooterL;
    private CANSparkMax _shooterR;

    // Motor Controller PIDs
    private SparkPIDController _shooterAngleController;
    private SparkPIDController _elevatorController;
    private SparkPIDController _shooterController;

    // Jukebox State Control
    private JukeboxEnum jukeboxCurrentState = JukeboxEnum.IDLE;
    private JukeboxEnum jukeboxPreviousState;

    // Elevator Profile
    private ElevatorFeedforward _elevatorFeedForward;
    private TrapezoidProfile _elevatorProfile;
    private TrapezoidProfile.State _elevatorProfileGoal;
    private TrapezoidProfile.State _elevatorProfileSetpoint;
    
    // Shooter Angle Profile
    private ArmFeedforward _shooterAngleFeedForward;
    private TrapezoidProfile _shooterAngleProfile;
    private TrapezoidProfile.State _shooterAngleProfileGoal;
    private TrapezoidProfile.State _shooterAngleProfileSetpoint;
    
    // Shooter Profile
    private SimpleMotorFeedforward _shooterFeedForward;
    private TrapezoidProfile _shooterProfile;
    private TrapezoidProfile.State _shooterProfileGoal;
    private TrapezoidProfile.State _shooterProfileSetpoint;

    // Feeder Note Control
    private DigitalInput _noteHolderPE;
    private DigitalInput _noteShooterPE;
    private Debouncer _noteHolderDebouncer;
    private Debouncer _noteShooterDebouncer;
    private Boolean _noteHold;
    private Boolean _noteShoot;
;
    // Set Points
    private final double kTrapElevatorPosition = 0; // change this
    private final double kExtendClimbElevatorPosition = 0; // change this
    private final double kClimbElevatorPosition = 0; // change this
    private final double kAmpElevatorPosition = 60;
    private final double kAmpShooterAngle = 5;

    // Elevator Control Constants
    private final double kElevatorMaxVelocity = 230; // change
    private final double kElevatorMaxAcceleration = 230; // change
    private final double ElevatorFeedforwardkS = 0.23312;
    private final double ElevatorFeedforwardkV = 0.11903;
    private final double ElevatorFeedforwardkG = 0.12293;
    private final double ElevatorFeedforwardkP = 0.001;
    
    // Shooter Angle Control Constants
    private final double kShooterAngleMaxVelocity = 50; // change
    private final double kShooterAngleMaxAcceleration = 50; // change
    private final double ShooterAngleFeedforwardkS = 0.31777; // change
    private final double ShooterAngleFeedforwardkV = 0.090231; // change
    private final double ShooterAngleFeedforwardkG = 0.035019; // change
    private final double ShooterAngleFeedforwardkP = 0.001; // change
    private final double ShooterAngleFeedforwardAngle = .1785; // change
    
    // Shooter Control Constants
    private final double kShooterMaxVelocity = 0; // change
    private final double kShooterMaxAcceleration = 0; // change
    private final double ShooterFeedforwardkS = 0.0; // change
    private final double ShooterFeedforwardkV = 0.0; // change
    private final double ShooterFeedforwardkG = 0.0; // change
    private final double ShooterFeedforwardkP = 0.0; // change

    // private final double kMaxOutput = 1.00;
    // private final double kMinOutput = -1.00;
    // private final double kMaxVel = 5;
    // private final double kMaxAccel = 5;
    // private final double kAllowedError = 3;
    // private final double speedToHoldElevator = 0.0;
    private double _manualElevatorSpeed;
    private double _manualFeederSpeed;
    private double _manualShooterAngleSpeed;
    private double _manualShooterSpeed;

    private VisionSystem _visionSystem;

    private final double[] kDistanceIDs = {};
    private final double[] kShooterAngles = {};
    private final double[] kShooterSpeeds = {};

    public Jukebox()
    {
        // Config Elevator
        configElevator();

        // Config Shooter Angle
        configShooterAngle();

        // Config Shooter
        configShooter();

        // Config Feeder
        configFeeder();

        _visionSystem = VisionSystem.getInstance();
        
        jukeboxPreviousState = JukeboxEnum.IDLE;
    }

    /**
     * Configures the Feeder Controllers and Sensors
     */
    private void configFeeder() {
        // feeder motor setup
        _feeder = new CANSparkMax(Constants.kFeederId, MotorType.kBrushless);
        _feeder.setIdleMode(IdleMode.kBrake);
        _feeder.setInverted(true);
        _feeder.burnFlash();

        _noteHolderPE = new DigitalInput(1);
        _noteShooterPE = new DigitalInput(2);

        /**
         * Rising (default): Debounces rising edges (transitions from false to true) only.
         * Falling: Debounces falling edges (transitions from true to false) only.
         * Both: Debounces all transitions.
         */
        _noteHolderDebouncer = new Debouncer(0.04, DebounceType.kRising);
        _noteShooterDebouncer = new Debouncer(0.04, DebounceType.kRising);
    }

    /**
     * Configures the Shooter Controllers and Profiling Constraints
     */
    private void configShooter() {
        // left shooter motor setup
        _shooterL = new CANSparkMax(Constants.kShooterLeftMotorId, MotorType.kBrushless);
        _shooterL.setIdleMode(IdleMode.kCoast);
        _shooterL.setSmartCurrentLimit(80, 100);
        _shooterL.setInverted(true);
        _shooterL.burnFlash();

        // right shooter motor setup
        _shooterR = new CANSparkMax(Constants.kShooterRightMotorId, MotorType.kBrushless);
        _shooterR.setIdleMode(IdleMode.kCoast);
        _shooterR.setSmartCurrentLimit(80, 100);
        _shooterR.setInverted(false);
        _shooterR.burnFlash();

        _shooterController = _shooterL.getPIDController();
        _shooterController.setP(ShooterFeedforwardkP);
        _shooterController.setI(0);
        _shooterController.setD(.001);
        _shooterController.setIZone(0);
        _shooterController.setFF(0);
        _shooterController.setOutputRange(0, 1.0);

        // creates the feed foward for the shooter
        _shooterFeedForward = new SimpleMotorFeedforward(ShooterFeedforwardkS, ShooterFeedforwardkG, ShooterFeedforwardkV);

        // the constraints our shooter has
        var shooterProfileConstraints = new TrapezoidProfile.Constraints(kShooterMaxVelocity, kShooterMaxAcceleration);
        _shooterProfile = new TrapezoidProfile(shooterProfileConstraints);

        // our desired state and current state
        _shooterProfileGoal = new TrapezoidProfile.State();
        _shooterProfileSetpoint = new TrapezoidProfile.State();
        
        _manualShooterSpeed = 0.0;
    }

    /**
     * Configures the Elevator Controllers and Profiling Constraints
     */
    private void configElevator() {
        // left elevator motor setup
        _elevatorL = new CANSparkMax(Constants.kLElevatorId, MotorType.kBrushless);
        _elevatorL.setIdleMode(IdleMode.kBrake);
        _elevatorL.setSmartCurrentLimit(20, 100);
        _elevatorL.setInverted(true);
        _elevatorL.burnFlash();
        
        // right elevator motor setup
        _elevatorR = new CANSparkMax(Constants.kRElevatorId, MotorType.kBrushless);
        _elevatorR.setIdleMode(IdleMode.kBrake);
        _elevatorR.setSmartCurrentLimit(20, 100);
        _elevatorR.setInverted(false);
        _elevatorR.burnFlash();
        
        _elevatorController = _elevatorL.getPIDController();
        _elevatorController.setP(ElevatorFeedforwardkP);
        _elevatorController.setI(0);
        _elevatorController.setD(.001);
        _elevatorController.setIZone(0);
        _elevatorController.setFF(0);
        _elevatorController.setOutputRange(-1.0, 1.0);

        // creates the feed foward for the elevator
        _elevatorFeedForward = new ElevatorFeedforward(ElevatorFeedforwardkS, ElevatorFeedforwardkG, ElevatorFeedforwardkV);

        // the constraints our elevator has
        var elevatorProfileConstraints = new TrapezoidProfile.Constraints(kElevatorMaxVelocity, kElevatorMaxAcceleration);
        _elevatorProfile = new TrapezoidProfile(elevatorProfileConstraints);

        // our desired state and current state
        _elevatorProfileGoal = new TrapezoidProfile.State();
        _elevatorProfileSetpoint = new TrapezoidProfile.State();
        
        _manualElevatorSpeed = 0.0;
    }
    
    /**
     * Configures the Shooter Angle Controllers and Profiling Constraints
     */
    private void configShooterAngle() {
        // shooter angle motor setup
        _shooterAngle = new CANSparkMax(Constants.kShooterAngleId, MotorType.kBrushless);
        _shooterAngle.setIdleMode(IdleMode.kBrake);
        _shooterAngle.setSmartCurrentLimit(20, 100);
        _shooterAngle.setInverted(false);
        _shooterAngle.burnFlash();

        _shooterAngleController = _shooterAngle.getPIDController();
        _shooterAngleController.setP(ShooterAngleFeedforwardkP);
        _shooterAngleController.setI(0);
        _shooterAngleController.setD(.001);
        _shooterAngleController.setIZone(0);
        _shooterAngleController.setFF(0);
        _shooterAngleController.setOutputRange(-1.0, 1.0);
        
        // creates the feed foward for the shooter angle
        _shooterAngleFeedForward = new ArmFeedforward(ShooterAngleFeedforwardkS, ShooterAngleFeedforwardkG, ShooterAngleFeedforwardkV);

        // the constraints our shooter angle has
        var shooterAngleProfileConstraints = new TrapezoidProfile.Constraints(kShooterAngleMaxVelocity, kShooterAngleMaxAcceleration);
        _shooterAngleProfile = new TrapezoidProfile(shooterAngleProfileConstraints);

        // our desired state and current state
        _shooterAngleProfileGoal = new TrapezoidProfile.State();
        _shooterAngleProfileSetpoint = new TrapezoidProfile.State();

        _manualShooterAngleSpeed = 0.0;
    }

    public static Jukebox getInstance()
    {
        if (_instance == null)
        {
            _instance = new Jukebox();
        }

        return _instance;
    }
    
    /**
     * Handles the position of the elevator using a Trapezoidal Motion Profile
     * The value should be set using setElevatorPosition. 
     * This method should be called during handleCurrentState()
     */
    private void handleElevatorPosition() {
        // This is the correct logic per the example at https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatortrapezoidprofile/Robot.java
        // t = time since last update, _elevatorProfileSetpoint is last state, _goal is the target state
        _elevatorProfileSetpoint = _elevatorProfile.calculate(0.02, _elevatorProfileSetpoint, _elevatorProfileGoal);

        // Check that this value is high enough for it to move.
        var ff = _elevatorFeedForward.calculate(_elevatorProfileSetpoint.velocity);

        // Test this as is. If this doesn't move, then multiply ff by 12 to get Volts. SetReference is expecting a voltage for the FF value here.
        // Just a note, I think our ElevatorFeedfowardkV value is a little bit low. The elevator for last year was .066 and this one is .001 This could be the cause. We can tune this if needed.
        _elevatorController.setReference(_elevatorProfileSetpoint.position, CANSparkBase.ControlType.kPosition, 0, ff);

        // This is being printed to Shuffleboard
        // Check that these position/velocity values are changing toward the setpoint, 5 in our test case.
        // Should be going closer to the target
        SmartDashboard.putNumber("profileSetpointPosition", _elevatorProfileSetpoint.position);

        // Should be going up then slowing down closer to the target
        SmartDashboard.putNumber("profileSetpointVelocity", _elevatorProfileSetpoint.velocity);
        
        // You can display these 3 values as a graph to see how they are proceeding.
        SmartDashboard.putNumber("elevatorFeedforward", ff);
    }
    
    /**
     * Handles the position of the shooter angle using a Trapezoidal Motion Profile
     * The value should be set using setShooterAngle. 
     * This method should be called during handleCurrentState()
     */
    private void handleShooterAnglePosition() {
        // t = time since last update, _shooterAngleProfileSetpoint is last state, _goal is the target state
        _shooterAngleProfileSetpoint = _shooterAngleProfile.calculate(0.02, _shooterAngleProfileSetpoint, _shooterAngleProfileGoal);

        var ff = _shooterAngleFeedForward.calculate(ShooterAngleFeedforwardAngle + _shooterAngleProfileSetpoint.position, _shooterAngleProfileSetpoint.velocity);

        _shooterAngleController.setReference(_shooterAngleProfileSetpoint.position, CANSparkBase.ControlType.kPosition, 0, ff);

        SmartDashboard.putNumber("angleProfileSetpointPosition", _shooterAngleProfileSetpoint.position);
        SmartDashboard.putNumber("angleProfileSetpointVelocity", _shooterAngleProfileSetpoint.velocity);
        SmartDashboard.putNumber("shooterAngleFeedforward", ff);
    }
    
    /**
     * Handles the velocity of the Shooter using a Trapezoidal Motion Profile
     * The value should be set using setShooterSpeed. 
     * This method should be called during handleCurrentState()
     */
    private void handleShooterSpeed() {
        // t = time since last update, _shooterProfileSetpoint is last state, _goal is the target state
        _shooterProfileSetpoint = _shooterProfile.calculate(0.02, _shooterProfileSetpoint, _shooterProfileGoal);

        // Multiply FF by 12 for voltage - Test this
        var ff = _shooterFeedForward.calculate(_shooterProfileSetpoint.velocity);

        _shooterAngleController.setReference(_shooterProfileSetpoint.velocity, CANSparkBase.ControlType.kVelocity, 0, ff);

        SmartDashboard.putNumber("shooterFeedforward", ff);
    }

    /**
     * Method that will set the angle of the shooter.
     * This should also be apply to the tilt angle too.
     * Once the shooter angle is set it should auto apply the tilt angle.
     * 1 means it set it clockwise
     * -1 means it set it counterclockwise
     */
    private void setShooterAngle(double desiredPosition) {
        // _shooterAngleController.setReference(desiredPosition, com.revrobotics.CANSparkBase.ControlType.kPosition, 0);
        _shooterProfileGoal = new TrapezoidProfile.State(desiredPosition, 0);
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
        // Set the profile goal
        _elevatorProfileGoal = new TrapezoidProfile.State(position, 0);
    }

    private void score() {
        // If previous state is PREP_AMP or PREP_TRAP -> Reverses out the front of the robot.
        // If previous state is PREP_SPEAKER -> Forward into the shooter motors in the back.
        if (jukeboxPreviousState == JukeboxEnum.PREP_AMP || jukeboxPreviousState == JukeboxEnum.PREP_TRAP)
        {
            _feeder.set(-.6);
        } else if (jukeboxPreviousState == JukeboxEnum.PREP_SPEAKER)
        {
            _feeder.set(.6);
        }
    }

    private void prepAmp() {
        // setShooterSpeed(0);
        setShooterAngle(30.0);
        setElevatorPosition(kAmpElevatorPosition); 
    }

    private void prepSpeaker() {
        setElevatorPosition(0);

        double targetDistance = _visionSystem.getDistance();
        if (targetDistance != 0.0) {
            double desiredShooterAngle = OneDimensionalLookup.interpLinear(
                kDistanceIDs,
                kShooterAngles,
                targetDistance
            );
            double desiredShooterSpeed = OneDimensionalLookup.interpLinear(
                kDistanceIDs,
                kShooterSpeeds,
                targetDistance
            );
            setShooterAngle(desiredShooterAngle);
            setShooterSpeed(desiredShooterSpeed);

        } else {
            setShooterAngle(Constants.KMinShooterAngle);
            setShooterSpeed(Constants.kMaxShooterSpeed);
        }
    }

    private void prepTrap() {
        if (jukeboxPreviousState == JukeboxEnum.CLIMB) {
            setElevatorPosition(kTrapElevatorPosition);
            setShooterAngle(-0.5);
            setShooterSpeed(0.0);
        }
    }

    private void reset() {
        setShooterAngle(0.0);
        setShooterSpeed(0.0);
    }

    private void extendForClimb() {
        setElevatorPosition(kExtendClimbElevatorPosition);
    }

    private void climb() {
        if (jukeboxPreviousState == JukeboxEnum.EXTEND_FOR_CLIMB) {
            setElevatorPosition(kClimbElevatorPosition);
        }
    }

    private void feeder()
    {
        if (!_noteShoot) {
            _feeder.set(-.2); 
        } else if (!_noteHold) {
            _feeder.set(0);
        } else {
            _feeder.set(.7);
        }
    }

    public void setManualFeederSpeed(double givenSpeed)
    {
        _manualFeederSpeed = givenSpeed;
    }

    private void idle() {
        setElevatorPosition(.5);
        setShooterAngle(0);
        setShooterSpeed(0.0);
        feeder();
    }

    private void manual() {
        _elevatorL.set(_manualElevatorSpeed);
        _shooterL.set(_manualShooterSpeed);
        _shooterAngle.set(_manualShooterAngleSpeed);
        _feeder.set(_manualFeederSpeed);
    }

    public void setManualElevatorSpeed(double givenSpeed)
    {
        _manualElevatorSpeed = givenSpeed;
    }

    public void setManualShooterSpeed(double speed) {
        _manualShooterSpeed = speed;
    }

    public void setManualShooterAngleSpeed(double speed) {
        _manualShooterAngleSpeed = speed;
    }

    public void resetSensors()  {
        _elevatorL.getEncoder().setPosition(0.0);
        _shooterAngle.getEncoder().setPosition(0.0);
    }

    public boolean hasNote() {
        // get() returns false if blocked/detects note
        return !_noteHolderPE.get() || !_noteShooterPE.get();
    }

    public void handleCurrentState()
    {
        switch(jukeboxCurrentState) {
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

        switch (jukeboxCurrentState) {
            case MANUAL:
                break;
            default:
                //handleShooterSpeed();
                handleShooterAnglePosition();
                handleElevatorPosition();
                break;
        }
    }

    public void setState(JukeboxEnum n)
    {
        // checks to see if the current state is what we are trying to set the state to
        if (n != jukeboxCurrentState) {
            switch (jukeboxCurrentState) {
                case CLIMB: 
                    if (n == JukeboxEnum.EXTEND_FOR_CLIMB || n == JukeboxEnum.PREP_TRAP)
                    {
                        jukeboxPreviousState = jukeboxCurrentState;
                        jukeboxCurrentState = n;
                    }
                    break;
                default:
                        jukeboxPreviousState = jukeboxCurrentState;
                        jukeboxCurrentState = n;
                    break;            
            }
        }
    }

    public JukeboxEnum getState() {
        return jukeboxCurrentState;
    }

    public void logTelemetry() {
        SmartDashboard.putNumber("Elevator motor left enconder position", _elevatorL.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor left enconder velocity", _elevatorL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator motor left temp", _elevatorL.getMotorTemperature());
        SmartDashboard.putNumber("Elevator motor left speed", _elevatorL.get());

        SmartDashboard.putNumber("Elevator motor right enconder position", _elevatorR.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor right enconder velocity", _elevatorR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator motor right temp", _elevatorR.getMotorTemperature());
        SmartDashboard.putNumber("Elevator motor right speed", _elevatorR.get());

        SmartDashboard.putNumber("Feeder motor enconder position", _feeder.getEncoder().getPosition());
        SmartDashboard.putNumber("Feeder motor enconder velocity", _feeder.getEncoder().getVelocity());
        SmartDashboard.putNumber("Feeder motor temp", _feeder.getMotorTemperature());
        SmartDashboard.putNumber("Feeder motor speed", _feeder.get());
        
        SmartDashboard.putNumber("Shooter angle motor enconder position", _shooterAngle.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter angle motor enconder velocity", _shooterAngle.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter angle motor temp", _shooterAngle.getMotorTemperature());
        SmartDashboard.putNumber("Shooter angle motor speed", _shooterAngle.get());

        SmartDashboard.putNumber("Shooter motor left enconder position", _shooterL.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter motor left enconder velocity", _shooterL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter motor left temp", _shooterL.getMotorTemperature());
        SmartDashboard.putNumber("Shooter motor left speed", _shooterL.get());

        SmartDashboard.putNumber("Shooter motor right enconder position", _shooterR.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter motor right enconder velocity", _shooterR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter motor right  temp", _shooterR.getMotorTemperature());
        SmartDashboard.putNumber("Shooter motor right speed", _shooterR.get());

        SmartDashboard.putBoolean("is the note pass the shooter limit switch", _noteShooterPE.get());
        SmartDashboard.putBoolean("is the note in the correct position in the holder", _noteHolderPE.get());

        SmartDashboard.putString("Jukebox current state", jukeboxCurrentState.toString());
        SmartDashboard.putString("Jukebox previous state", jukeboxPreviousState.toString());


        _noteHold = _noteHolderDebouncer.calculate(_noteHolderPE.get());
        _noteShoot = _noteShooterDebouncer.calculate(_noteShooterPE.get());
    }
}