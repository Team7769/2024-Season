package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Utilities.LEDController;
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

    private double _targetDistance = 0;

    // Motor Controller PIDs
    private SparkPIDController _shooterAngleController;
    private SparkPIDController _elevatorController;
    private SparkPIDController _shooterController;
    private SparkPIDController _shooterRController;

    // Jukebox State Control
    private JukeboxEnum jukeboxCurrentState = JukeboxEnum.IDLE;
    private JukeboxEnum jukeboxPreviousState = JukeboxEnum.IDLE;

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
    private double _shooterSetpoint;

    // Feeder Note Control
    private final int kNoteHolderPEChannel = 1;
    private final int kNoteShooterPEChannel = 2;

    private DigitalInput _noteHolderPE;
    private DigitalInput _noteShooterPE;
    private Debouncer _noteHolderPEDebouncer;
    private Debouncer _noteShooterPEDebouncer;
    private Boolean _inNoteHolder = false;
    private Boolean _inNoteShooter = false;

    private final double kPhotoEyeDebounceTime = 0.04;

    // Set Points
    private final double kTrapShooterAngle = 9;
    private final double kTrapElevatorPosition = 80; // change this
    private final double kExtendClimbElevatorPosition = 98; // change this
    private final double kExtendClimbShooterAngle = 4;
    private final double kAmpElevatorPosition = 70;
    private final double kFeedShooterAngle = 7;
    private final double kPodiumSpeakerShotAngle = 5.9;
    private final double kPodiumSpeakerShotSpeed = 38;
    private final double kLineSpeakerShotAngle = 5.2;
    private final double kLineSpeakerShotSpeed = 35;
    private final double kHumanElementIntakeAngle = 9;
    private final double kEmergancyEjectElevatorPosition = 10;
    private final double kLaunchAngle = 11;
    private final double kLaunchSpeed = 48;

    // Elevator Control Constants
    private final double kElevatorMaxVelocity = 230;
    private final double kElevatorMaxAcceleration = 230;
    private final double kElevatorFeedForwardKs = 0.23312;
    private final double kElevatorFeedForwardKv = 0.11903;
    private final double kElevatorFeedForwardKg = 0.12293;
    private final double kElevatorFeedForwardKp = 0.01;
    
    // Shooter Angle Control Constants
    private final double kShooterAngleMaxVelocity = 50;
    private final double kShooterAngleMaxAcceleration = 50;
    private final double kShooterAngleFeedForwardKs = 0.31777;
    private final double kShooterAngleFeedForwardKv = 0.090231;
    private final double kShooterAngleFeedForwardkG = 0.035019;
    private final double kShooterAngleFeedForwardKp = 0.075;
    private final double kShooterAngleFeedForwardAngle = .1785;
    
    // Shooter Control Constants
    private final double kShooterFeedForwardKs = 0.37431;
    private final double kShooterFeedForwardKv = 0.0023755;
    
    private final double kShooterFeedForwardKp = .001;

    private final int kLowStallLimit = 20;
    private final int kHighStallLimit = 80;
    private final int kFreeLimit = 100;

    private final double kCommonKd = 0.001;
    private final double kTInterval = 0.02;

    private final double kFeederShootSpeed = 0.5;
    private final double kFeederReverse = -0.2;
    private final double kFeederIntake = 0.25;

    private final double[] kDistanceIDs = {2, 2.5, 3, 3.5, 4};
    private final double[] kShooterAngles = {5, 5.65, 6.2, 6.55, 6.6};
    private final double[] kShooterSpeeds = {35, 36, 38, 41, 44};

    private double _manualElevatorSpeed = 0;
    private double _manualFeederSpeed = 0;
    private double _manualShooterAngleSpeed = 0;
    private double _manualShooterSpeed = 0;

    private VisionSystem _visionSystem;

    private double _dashboardShooterTargetSpeed = 0.0;
    private double _dashboardShooterTargetAngle = 0.0;
    private double _dashboardShooterRPercent =  1.0;
    private double _shooterSetpointRpm = 0.0;

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

        _noteHolderPE = new DigitalInput(kNoteHolderPEChannel);
        _noteShooterPE = new DigitalInput(kNoteShooterPEChannel);

        /**
         * Rising (default): Debounces rising edges (transitions from false to true) only.
         * Falling: Debounces falling edges (transitions from true to false) only.
         * Both: Debounces all transitions.
         */
        _noteHolderPEDebouncer = new Debouncer(kPhotoEyeDebounceTime,
                                               DebounceType.kRising);

        _noteShooterPEDebouncer = new Debouncer(kPhotoEyeDebounceTime,
                                                DebounceType.kRising);
    }

    /**
     * Configures the Shooter Controllers and Profiling Constraints
     */
    private void configShooter() {
        // left shooter motor setup
        _shooterL = new CANSparkMax(Constants.kShooterLeftMotorId,
                                    MotorType.kBrushless);

        _shooterL.setIdleMode(IdleMode.kCoast);
        _shooterL.setSmartCurrentLimit(kHighStallLimit, kFreeLimit);
        _shooterL.setInverted(true);
        _shooterL.burnFlash();

        // right shooter motor setup
        _shooterR = new CANSparkMax(Constants.kShooterRightMotorId,
                                    MotorType.kBrushless);

        _shooterR.setIdleMode(IdleMode.kCoast);
        _shooterR.setSmartCurrentLimit(kHighStallLimit, kFreeLimit);
        _shooterR.setInverted(false);
        _shooterR.burnFlash();

        _shooterController = _shooterL.getPIDController();
        _shooterController.setP(kShooterFeedForwardKp);
        _shooterController.setI(0);
        _shooterController.setD(kCommonKd);
        _shooterController.setIZone(0);
        _shooterController.setFF(0);
        _shooterController.setOutputRange(0, 1.0);

        _shooterRController = _shooterR.getPIDController();
        _shooterRController.setP(kShooterFeedForwardKp);
        _shooterRController.setI(0);
        _shooterRController.setD(kCommonKd);
        _shooterRController.setIZone(0);
        _shooterRController.setFF(0);
        _shooterRController.setOutputRange(0, 1.0);

        // creates the feed foward for the shooter
        _shooterFeedForward = new SimpleMotorFeedforward(
            kShooterFeedForwardKs,
            kShooterFeedForwardKv
        );
        
        _manualShooterSpeed = 0.0;
        _shooterSetpoint = 0.0;
    }

    /**
     * Configures the Elevator Controllers and Profiling Constraints
     */
    private void configElevator() {
        // left elevator motor setup
        _elevatorL = new CANSparkMax(Constants.kLElevatorId,
                                     MotorType.kBrushless);

        _elevatorL.setIdleMode(IdleMode.kBrake);
        _elevatorL.setSmartCurrentLimit(kLowStallLimit, kFreeLimit);
        _elevatorL.setInverted(true);
        _elevatorL.burnFlash();
        
        // right elevator motor setup
        _elevatorR = new CANSparkMax(Constants.kRElevatorId,
                                     MotorType.kBrushless);

        _elevatorR.setIdleMode(IdleMode.kBrake);
        _elevatorR.setSmartCurrentLimit(kLowStallLimit, kFreeLimit);
        _elevatorR.setInverted(false);
        _elevatorR.burnFlash();
        
        _elevatorController = _elevatorL.getPIDController();
        _elevatorController.setP(kElevatorFeedForwardKp);
        _elevatorController.setI(0);
        _elevatorController.setD(kCommonKd);
        _elevatorController.setIZone(0);
        _elevatorController.setFF(0);
        _elevatorController.setOutputRange(-1.0, 1.0);

        // creates the feed foward for the elevator
        _elevatorFeedForward = new ElevatorFeedforward(
            kElevatorFeedForwardKs,
            kElevatorFeedForwardKg,
            kElevatorFeedForwardKv
        );

        Constraints elevatorProfileConstraints;

        // the constraints our elevator has
        elevatorProfileConstraints = new TrapezoidProfile.Constraints(
            kElevatorMaxVelocity,
            kElevatorMaxAcceleration
        );

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
        _shooterAngle = new CANSparkMax(Constants.kShooterAngleId,
                                        MotorType.kBrushless);

        _shooterAngle.setIdleMode(IdleMode.kBrake);
        _shooterAngle.setSmartCurrentLimit(kLowStallLimit, kFreeLimit);
        _shooterAngle.setInverted(false);
        _shooterAngle.burnFlash();

        _shooterAngleController = _shooterAngle.getPIDController();
        _shooterAngleController.setP(kShooterAngleFeedForwardKp);
        _shooterAngleController.setI(0);
        _shooterAngleController.setD(kCommonKd);
        _shooterAngleController.setIZone(0);
        _shooterAngleController.setFF(0);
        _shooterAngleController.setOutputRange(-1.0, 1.0);
        
        // creates the feed foward for the shooter angle
        _shooterAngleFeedForward = new ArmFeedforward(
            kShooterAngleFeedForwardKs,
            kShooterAngleFeedForwardkG,
            kShooterAngleFeedForwardKv
        );

        Constraints shooterAngleProfileConstraints;

        // the constraints our shooter angle has
        shooterAngleProfileConstraints = new TrapezoidProfile.Constraints(
            kShooterAngleMaxVelocity,
            kShooterAngleMaxAcceleration
        );

        _shooterAngleProfile = new TrapezoidProfile(
            shooterAngleProfileConstraints
        );

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
        _elevatorProfileSetpoint = _elevatorProfile.calculate(
            kTInterval,
            _elevatorProfileSetpoint,
            _elevatorProfileGoal
        );

        // Check that this value is high enough for it to move.
        double calculatedFeedForward = _elevatorFeedForward.calculate(
            _elevatorProfileSetpoint.velocity
        );

        // Test this as is. If this doesn't move, then multiply ff by 12 to get Volts. SetReference is expecting a voltage for the FF value here.
        // Just a note, I think our ElevatorFeedfowardkV value is a little bit low. The elevator for last year was .066 and this one is .001 This could be the cause. We can tune this if needed.
        if (_elevatorL.getEncoder().getPosition() >= 85 && _elevatorProfileSetpoint.velocity > 0) {
           _elevatorL.set(0);
        } else { 
            _elevatorController.setReference(_elevatorProfileSetpoint.position,
                            CANSparkBase.ControlType.kPosition,
                            0,
                            calculatedFeedForward);
        }

        // This is being printed to Shuffleboard
        // Check that these position/velocity values are changing toward the setpoint, 5 in our test case.
        // Should be going closer to the target
        SmartDashboard.putNumber("profileSetpointPosition", _elevatorProfileSetpoint.position);

        // Should be going up then slowing down closer to the target
        SmartDashboard.putNumber("profileSetpointVelocity", _elevatorProfileSetpoint.velocity);
        
        // You can display these 3 values as a graph to see how they are proceeding.
        SmartDashboard.putNumber("elevatorFeedforward", calculatedFeedForward);
    }
    
    /**
     * Handles the position of the shooter angle using a Trapezoidal Motion Profile
     * The value should be set using setShooterAngle. 
     * This method should be called during handleCurrentState()
     */
    private void handleShooterAnglePosition() {
        // t = time since last update, _shooterAngleProfileSetpoint is last state, _goal is the target state
        _shooterAngleProfileSetpoint = _shooterAngleProfile.calculate(
            kTInterval,
            _shooterAngleProfileSetpoint,
            _shooterAngleProfileGoal
        );

        double calculatedFeedForward = _shooterAngleFeedForward.calculate(
            kShooterAngleFeedForwardAngle +
            _shooterAngleProfileSetpoint.position,
            _shooterAngleProfileSetpoint.velocity
        );

        _shooterAngleController.setReference(
            _shooterAngleProfileSetpoint.position,
            CANSparkBase.ControlType.kPosition,
            0,
            calculatedFeedForward);

        SmartDashboard.putNumber("angleProfileSetpointPosition",
                                 _shooterAngleProfileSetpoint.position);

        SmartDashboard.putNumber("angleProfileSetpointVelocity",
                                 _shooterAngleProfileSetpoint.velocity);

        SmartDashboard.putNumber("shooterAngleFeedforward",
                                 calculatedFeedForward);

    }
    
    /**
     * Handles the velocity of the Shooter using a Trapezoidal Motion Profile
     * The value should be set using setShooterSpeed. 
     * This method should be called during handleCurrentState()
     */
    private void handleShooterSpeed() {
        double calculatedFeedForward = _shooterFeedForward.calculate(
            _shooterSetpoint
        );
        double calculatedFeedForwardR = _shooterFeedForward.calculate(_shooterSetpoint * _dashboardShooterRPercent);

        _shooterSetpointRpm = _shooterSetpoint * 60;

        _shooterController.setReference(_shooterSetpointRpm,
                                        CANSparkBase.ControlType.kVelocity,
                                        0,
                                        calculatedFeedForward);
        _shooterRController.setReference(_shooterSetpointRpm * _dashboardShooterRPercent,
                                        CANSparkBase.ControlType.kVelocity,
                                        0,
                                        calculatedFeedForwardR);
        SmartDashboard.putNumber("shooterSetpointRpm", _shooterSetpointRpm);
        SmartDashboard.putNumber("shooterFeedforward", calculatedFeedForward);
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
        _shooterAngleProfileGoal = new TrapezoidProfile.State(desiredPosition,
                                                              0);
    }

    /**
     * Sets the elevator to where it needs to be and if the position changes we reset the timer and update the old position to the new position
     * @param position takes a double and makes the goal state.
     */
    private void setElevatorPosition(double desiredPosition)
    {
        // Set the profile goal
        _elevatorProfileGoal = new TrapezoidProfile.State(desiredPosition, 0);
    }

    /**
     * ShooterDeploy ramps up the shooter motors to the max
     */
    private void setShooterSpeed(double v)
    {
        _shooterSetpoint = v;
    }

    private void score() {
        // If previous state is PREP_AMP or PREP_TRAP -> Reverses out the front of the robot.
        // If previous state is PREP_SPEAKER -> Forward into the shooter motors in the back.
        if (jukeboxPreviousState == JukeboxEnum.PREP_AMP ||
            jukeboxPreviousState == JukeboxEnum.PREP_TRAP) {

            _feeder.set(-kFeederShootSpeed);
        } else if (jukeboxPreviousState == JukeboxEnum.PREP_SPEAKER || 
                    jukeboxPreviousState == JukeboxEnum.PREP_SPEAKER_PODIUM ||
                    jukeboxPreviousState == JukeboxEnum.PREP_SPEAKER_LINE ||
                    jukeboxPreviousState == JukeboxEnum.PREP_LAUNCH) {
            _feeder.set(kFeederShootSpeed);
        }
    }

    private void prepAmp() {
        feeder();
        setShooterSpeed(0);
        setShooterAngle(kFeedShooterAngle);
        setElevatorPosition(kAmpElevatorPosition); 
    }

    private void prepTrap() {
        if (jukeboxPreviousState != JukeboxEnum.CLIMB) return;

        _feeder.set(kFeederIntake);

        setShooterSpeed(0.0);

        var elevatorPosition = _elevatorL.getEncoder().getPosition();
        if (elevatorPosition < 3) {
            setElevatorPosition(5);
        } else if (elevatorPosition >= 3) {
            setShooterAngle(kTrapShooterAngle);
            if (_shooterAngle.getEncoder().getPosition() >= 7)
            {
                setElevatorPosition(kTrapElevatorPosition);
            }
        }
    }

    private void prepSpeaker() {
        setElevatorPosition(0);

        _targetDistance = _visionSystem.getDistance();

        feeder();

        if (_targetDistance != 0.0) {
            double desiredShooterAngle = OneDimensionalLookup.interpLinear(
                kDistanceIDs,
                kShooterAngles,
                _targetDistance
            );
            double desiredShooterSpeed = OneDimensionalLookup.interpLinear(
                kDistanceIDs,
                kShooterSpeeds,
                _targetDistance
            );
            setShooterAngle(desiredShooterAngle);
            setShooterSpeed(desiredShooterSpeed);

        } else {
            setShooterAngle(Constants.KMinShooterAngle);
            setShooterSpeed(Constants.kMaxShooterSpeed);
        }
    }

    /** Preps the speaker for a shot from the podium (Doesn't use auto aim) */
    private void prepSpeakerPodium() {
        setShooterAngle(kPodiumSpeakerShotAngle);
        setShooterSpeed(kPodiumSpeakerShotSpeed);
        setElevatorPosition(0);
    }

    private void prepSpeakerLine() {
        setShooterAngle(kLineSpeakerShotAngle);
        setShooterSpeed(kLineSpeakerShotSpeed);
        setElevatorPosition(0);
    }

    private void prepLaunch() {
        setShooterAngle(kLaunchAngle);
        setShooterSpeed(kLaunchSpeed);
    }

    private void prepHumanIntake() {
        setShooterAngle(kHumanElementIntakeAngle);
        setElevatorPosition(0);
    }

    private void reset() {
        setShooterAngle(0.0);
        setShooterSpeed(0.0);
    }

    private void extendForClimb() {
        setElevatorPosition(kExtendClimbElevatorPosition);
        setShooterAngle(kExtendClimbShooterAngle);
    }

    private void climb() {
        if (jukeboxPreviousState != JukeboxEnum.EXTEND_FOR_CLIMB) {
            return;
        }
        setElevatorPosition(.05);
        _elevatorProfileSetpoint = new TrapezoidProfile.State();
        if (_elevatorL.getEncoder().getPosition() > .05) {
            _elevatorL.set(-.5);
        } else {
            _elevatorL.set(0);
        }
    }

    private void emergancyEject() {
        setElevatorPosition(kEmergancyEjectElevatorPosition);
    }

    private void feeder()
    {
        if (_inNoteShooter) {
            _feeder.set(kFeederReverse); 
        } else if (_inNoteHolder) {
            _feeder.set(0);
        } else {
            _feeder.set(kFeederIntake);
        }
    }

    public void setManualFeederSpeed(double givenSpeed)
    {
        _manualFeederSpeed = givenSpeed;
    }

    private void idle() {
        setElevatorPosition(0.5); // dont understand why .5
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

    private void jukeboxTest() {
        feeder();
        setShooterAngle(_dashboardShooterTargetAngle);
        setShooterSpeed(_dashboardShooterTargetSpeed);
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
        return _inNoteHolder || _inNoteShooter;
    }

    public double getElevatorPosition() {
        return _elevatorL.getEncoder().getPosition();
    }

    public int getShooterLeds(int numLeds) {
        var target = _shooterSetpointRpm + 300;
        if (target > 0) {
            double shooterProportion = numLeds * (_shooterL.getEncoder().getVelocity() / (target));
            return (int)Math.ceil(shooterProportion);
        } else {
            return 0;
        }

    }

    public void handleCurrentState()
    {
        switch(jukeboxCurrentState) {
            case MANUAL:
                manual();
                break;
            case JUKEBOX_TEST:
                jukeboxTest();
                break;
            case SCORE:
                score();
                break;
            case PREP_SPEAKER:
                prepSpeaker();
                break;
            case PREP_SPEAKER_PODIUM:
                prepSpeakerPodium();
                break;
            case PREP_SPEAKER_LINE:
                prepSpeakerLine();
                break;
            case PREP_LAUNCH:
                prepLaunch();
                break;
            case PREP_HUMAN_INTAKE:
                prepHumanIntake();
            case PREP_AMP:
                prepAmp();
                break;
            case PREP_TRAP:
                prepTrap();
                break;
            case RESET:
                reset();
            case EJECT:
                emergancyEject();
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
            case CLIMB:
                break;
            default:
                handleShooterSpeed();
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
                    if (n == JukeboxEnum.EXTEND_FOR_CLIMB ||
                        n == JukeboxEnum.PREP_TRAP) {
                        
                        jukeboxPreviousState = jukeboxCurrentState;
                        jukeboxCurrentState = n;
                    }
                    if (n == JukeboxEnum.EXTEND_FOR_CLIMB) {
                        
                        _elevatorProfileSetpoint = new TrapezoidProfile.State(getElevatorPosition(), 0);
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

    public JukeboxEnum getPreviousState() {
        return jukeboxPreviousState;
    }

    public boolean isReadyToScore() {
        switch (jukeboxCurrentState) {
            case PREP_AMP:
            case PREP_TRAP:
                // Logic if Prep Amp/Trap is ready to score
                break;
            case PREP_SPEAKER:
            case PREP_SPEAKER_PODIUM:
            case PREP_SPEAKER_LINE:
                // Error is the absolute value of the difference between Target and Actual 
                var shooterError = Math.abs((_shooterSetpointRpm + 300) - _shooterL.getEncoder().getVelocity());
                var angleError = Math.abs(_shooterAngleProfileSetpoint.position - _shooterAngle.getEncoder().getPosition());

                // TODO: These error numbers need to tuned/configured. 
                // We also may want a debouncer for the result of this method so that it must be ready to score for a minimum amount of time first.
                return (shooterError <= 150 && angleError <= .75);
            default:
                return false;
        }

        return false;
    }

    public void logTelemetry() {
        _dashboardShooterTargetAngle = SmartDashboard.getNumber("dashboardShooterTargetAngle", 0.0);
        _dashboardShooterTargetSpeed = SmartDashboard.getNumber("dashboardShooterTargetSpeed", 0.0);
        _dashboardShooterRPercent = SmartDashboard.getNumber("dashboardShooterRPercent", 1.0);
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


        _inNoteHolder = !_noteHolderPEDebouncer.calculate(
            _noteHolderPE.get()
        );

        _inNoteShooter = !_noteShooterPEDebouncer.calculate(
            _noteShooterPE.get()
        );

        SmartDashboard.putNumber("dashboardShooterTargetAngle", _dashboardShooterTargetAngle);
        SmartDashboard.putNumber("dashboardShooterTargetSpeed", _dashboardShooterTargetSpeed);
        SmartDashboard.putNumber("dashboardShooterRPercent", _dashboardShooterRPercent);
        SmartDashboard.putBoolean("shooterReady", isReadyToScore());
    }
}