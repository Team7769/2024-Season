package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Drivetrain extends Subsystem{
    private static Drivetrain _instance;

    private final SwerveModule _frontLeftModule;
    private final SwerveModule _frontRightModule;
    private final SwerveModule _backLeftModule;
    private final SwerveModule _backRightModule;

    private final SwerveDrivePoseEstimator _drivePoseEstimator;

    private SwerveModuleState[] _moduleStates = new SwerveModuleState[4];

    SwerveModulePosition[] _modulePositions = new SwerveModulePosition[4];
    // needs device id constant or port value
    // are we using pigeon2? example uses pigeon2
    private final Pigeon2 _gyro = new Pigeon2(Constants.kPigeonId);
    private double _gyroOffset = 0.0;

    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final Field2d m_field = new Field2d();

    private final String kLimelightName = "";

    // in meters
    private final double kNearTagAreaMin = 0.8;
    private final double kFarTagAreaMin = 0.1;

    private final double kMidTrustPoseDiffMax = 0.5;
    private final double kLowTrustPoseDiffMax = 0.3;

    // std = standard deviation
    // in meters
    private final double kHighTrustXYStds = 0.5;
    private final double kMidTrustXYStds = 1.0;
    private final double kLowTrustXYStds = 2.0;

    // rotation deviation
    // in degrees
    private final double kHighTrustDegStds = 6;
    private final double kMidTrustDegStds = 12;
    private final double kLowTrustDegStds = 30;

    private Drivetrain()
    {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        _frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Left Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.kFrontLeftDriveId)
            .withSteerMotor(MotorType.NEO, Constants.kFrontLeftSteerId)
            .withSteerEncoderPort(Constants.kFrontLeftSteerEncoderId)
            .withSteerOffset(Constants.kFrontLeftEncoderOffset)
            .build();

        _frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Right Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.kFrontRightDriveId)
            .withSteerMotor(MotorType.NEO, Constants.kFrontRightSteerId)
            .withSteerEncoderPort(Constants.kFrontRightSteerEncoderId)
            .withSteerOffset(Constants.kFrontRightEncoderOffset)
            .build();

        _backLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Left Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.kBackLeftDriveId)
            .withSteerMotor(MotorType.NEO, Constants.kBackLeftSteerId)
            .withSteerEncoderPort(Constants.kBackLeftSteerEncoderId)
            .withSteerOffset(Constants.kBackLeftEncoderOffset)
            .build();

        _backRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Right Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.kBackRightDriveId)
            .withSteerMotor(MotorType.NEO, Constants.kBackRightSteerId)
            .withSteerEncoderPort(Constants.kBackRightSteerEncoderId)
            .withSteerOffset(Constants.kBackRightEncoderOffset)
            .build();

        _drivePoseEstimator = new SwerveDrivePoseEstimator(
            Constants._kinematics,
            getGyroRotation(),
            new SwerveModulePosition[] {
                _frontLeftModule.getPosition(),
                _frontRightModule.getPosition(),
                _backLeftModule.getPosition(),
                _backRightModule.getPosition()
            },
            new Pose2d()
        );
        SmartDashboard.putData("Field",m_field);
    }

    public static Drivetrain getInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }

        return _instance;
    }

    // public void zeroSensors() {
    //     resetOdometry();
    // }

    public void logTelemetry()
    {
        var pose = _drivePoseEstimator.getEstimatedPosition();
        
        m_field.setRobotPose(pose);

        SmartDashboard.putNumber("drivetrainGyroAngle",
                                 getGyroRotation().getDegrees());
        SmartDashboard.putNumber("drivetrainGyroAngleWithOffset",
                                 getGyroRotationWithOffset().getDegrees());

        SmartDashboard.putNumber("drivetrainChassisSpeedsVx",
                                 _chassisSpeeds.vxMetersPerSecond);

        SmartDashboard.putNumber("drivetrainChassisSpeedsVy",
                                 _chassisSpeeds.vyMetersPerSecond);

        SmartDashboard.putNumber("drivetrainChassisSpeedsWz",
                                 _chassisSpeeds.omegaRadiansPerSecond);

        SmartDashboard.putNumber("drivetrainGyroOffset", _gyroOffset);
        SmartDashboard.putNumber("drivetrainPitch",
                                 _gyro.getRoll().getValueAsDouble());

        SmartDashboard.putNumber("drivetrainOdometryX", pose.getX());
        SmartDashboard.putNumber("drivetrainOdometryY", pose.getY());
        SmartDashboard.putNumber("drivetrainOdometryZ",
                                 pose.getRotation().getDegrees());
    }
    
    public void updateOdometry()
    {

        _drivePoseEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getGyroRotation(),
            new SwerveModulePosition[] {
                _frontLeftModule.getPosition(),
                _frontRightModule.getPosition(),
                _backLeftModule.getPosition(),
                _backRightModule.getPosition()
            }
        );

        PoseEstimate poseEstimate = LimelightHelpers
            .getBotPoseEstimate_wpiBlue(kLimelightName);

        if (poseEstimate.tagCount == 0) return;

        double poseDifference = _drivePoseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .getDistance(poseEstimate.pose.getTranslation());

        double xyStds;
        double degStds;
        
        // note that besides the tag count check, there are no equals checks 
        // (e.g. <=) just < and >, for var name consistency we use =
        if (poseEstimate.tagCount >= 2) {
            xyStds = kHighTrustXYStds;
            degStds = kHighTrustDegStds;

        } else if (poseEstimate.avgTagArea >= kNearTagAreaMin &&
                   poseDifference <= kMidTrustPoseDiffMax) {

            xyStds = kMidTrustXYStds;
            degStds = kMidTrustDegStds;

        } else if (poseEstimate.avgTagArea >= kFarTagAreaMin &&
                   poseDifference <= kLowTrustPoseDiffMax) {

            xyStds = kLowTrustXYStds;
            degStds = kLowTrustDegStds;
        } else {
            return;
        }

        _drivePoseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds))
        );

        _drivePoseEstimator.addVisionMeasurement(
            poseEstimate.pose,
            Timer.getFPGATimestamp() - poseEstimate.latency
        );
    }

    // public void resetOdometry()
    // {

    // }

    public Rotation2d getGyroRotation()
    {
        // return rotation2d with first method
        return _gyro.getRotation2d();
    }

    public Rotation2d getGyroRotationWithOffset()
    {
        // return rotation2d + offset with second method
        return Rotation2d.fromDegrees(_gyro.getRotation2d().getDegrees() +
                                      _gyroOffset);
    }
    /** Takes a list of SwerveModuleStates for each swerve modual and sets each swerve modual to that state (the angle and speed of the wheel).
     * 
     * @param moduleStates A list that contains the necessary state for each swerve modual from front left and right to back left and back right in that order.
     */
    private void setModuleStates(SwerveModuleState[] moduleStates)
    {
        // set voltage to deliver to motors and angle to rotate wheel to
        _frontLeftModule.set(moduleStates[0].speedMetersPerSecond /
                             Constants.MAX_VELOCITY_METERS_PER_SECOND *
                             Constants.MAX_VOLTAGE,
                             moduleStates[0].angle.getRadians());

        _frontRightModule.set(moduleStates[1].speedMetersPerSecond /
                              Constants.MAX_VELOCITY_METERS_PER_SECOND *
                              Constants.MAX_VOLTAGE,
                              moduleStates[1].angle.getRadians());

        _backLeftModule.set(moduleStates[2].speedMetersPerSecond /
                            Constants.MAX_VELOCITY_METERS_PER_SECOND *
                            Constants.MAX_VOLTAGE,
                            moduleStates[2].angle.getRadians());

        _backRightModule.set(moduleStates[3].speedMetersPerSecond /
                             Constants.MAX_VELOCITY_METERS_PER_SECOND *
                             Constants.MAX_VOLTAGE,
                             moduleStates[3].angle.getRadians());
    }

    /**
     * This method gets the SwerveModulePosition of each swerve modual and returns an array of them.
     * 
     * @return SwerveModulePosition[] returns an array of swerve module positions 
     */
    public SwerveModulePosition[] getModulePositions()
    {
        _modulePositions[0] = _frontLeftModule.getPosition();
        _modulePositions[1] = _frontRightModule.getPosition();
        _modulePositions[2] = _backLeftModule.getPosition();
        _modulePositions[3] = _backRightModule.getPosition();
        return _modulePositions;
    }
    /** Method that takes translations from the drive controller creates a chassisSpeed object and feeds it into the drive method
        Drive and fieldOrientedDrive are seperate due to autonomus getting chassis speed directly with no need to translate

        @param translationX Double value from the left joysticks vertical axis multiplied by max velocity as fromFieldRelativeSpeeds
        requires m/s (Used for translation)

        @param translationY Double value from the left joysticks horizontal axis once again multiplied by max velocity (Used for translation)

        @param rotationZ Double value from the right joysticks horizontal axis once again multiplied by max velocity (Used for rotation)
     */ 
    public void fieldOrientedDrive(double translationX,
                                   double translationY,
                                   double rotationZ)
    {   
        var alliance = DriverStation.getAlliance();
        var invert = alliance.isPresent() && alliance.get() == Alliance.Red ? -1 : 1; 
        _chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            invert * translationX * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            invert * translationY * Constants.MAX_VELOCITY_METERS_PER_SECOND, 
            rotationZ * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND, 
            getGyroRotationWithOffset()
        );
        
        SmartDashboard.putNumber("chassisSpeedsX", _chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("chassisSpeedsY", _chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("chassisSpeedsZ", _chassisSpeeds.omegaRadiansPerSecond);    

        drive(_chassisSpeeds);
    }

    /** Method that takes a ChassisSpeeds object and sets each swerve module to it's required state (position, speed, etc) also stores the last modual states applied.
     * 
     * @param chassisSpeeds A variable for a ChasisSpeeds object hold X, Y, and rotational velocity as well as the 2D rotation of the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds)
    {
        // Sets all the modules to their proper states
        var moduleStates = Constants
            ._kinematics
            .toSwerveModuleStates(chassisSpeeds);
        

        // normalize speed based on max velocity meters
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            Constants.MAX_VELOCITY_METERS_PER_SECOND
        );

        setModuleStates(moduleStates);
        _moduleStates = moduleStates;
        _chassisSpeeds = chassisSpeeds;
    }

    /** 
     * Method that resets the pigeon current direction the robot is facing will be the front
     */
    public void reset()
    {
        _gyro.setYaw(0);
    }

    public void setStartingPose(Pose2d startingPose) {
        _gyroOffset = startingPose.getRotation().getDegrees();
        SmartDashboard.putNumber("startingPoseRotation", startingPose.getRotation().getDegrees());
        _drivePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), startingPose);
    }

    public Pose2d getPose(){
        return _drivePoseEstimator.getEstimatedPosition();
    }
}
