package frc.robot.Constants;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    // Controller USB ports
    public static final int kDriverControllerUsbSlot = 0;
    public static final int kOperatorControllerUsbSlot = 1;

    // Swerve CAN IDs
    public static final int kFrontLeftDriveId = 2;
    public static final int kFrontLeftSteerId = 3;
    public static final int kFrontLeftSteerEncoderId = 4;
    
    public static final int kFrontRightDriveId = 5;
    public static final int kFrontRightSteerId = 6;
    public static final int kFrontRightSteerEncoderId = 7;
    
    public static final int kBackLeftDriveId = 8;
    public static final int kBackLeftSteerId = 9;
    public static final int kBackLeftSteerEncoderId = 10;

    public static final int kBackRightDriveId = 11;
    public static final int kBackRightSteerId = 12;
    public static final int kBackRightSteerEncoderId = 13;

    public static final int kPigeonId = 14;

    public static final int kIntakeMotorId = 16;
    public static final int kLElevatorId = 17;
    public static final int kRElevatorId = 18;
    public static final int kFeederId = 19;
    public static final int kShooterAngleId = 20;
    public static final int kShooterLeftMotorId = 21;
    public static final int kShooterRightMotorId = 22;
    
    
    public static final double kFrontLeftEncoderOffset = -Math.toRadians(85.86914);
    public static final double kFrontRightEncoderOffset = -Math.toRadians(348.57421);
    public static final double kBackLeftEncoderOffset = -Math.toRadians(219.55078);
    public static final double kBackRightEncoderOffset = -Math.toRadians(295.40039);

    public static final double kP = 0.015;
    public static final double kI = 0.0;
    public static final double kD = 0.001;
    public static final double kFF = 0.0;
    public static final double kIz = 0.0;
    public static final double kMaxOutput = 1.00;
    public static final double kMinOutput = -1.00;
    public static final double kMaxVel = 10;
    public static final double kMaxAccel = 10;
    public static final double kAllowedError = 3;

    public static final double speedToHoldElevator = 0.0;
    public static final double kMaxElevatorHeight = 82.0;
    public static final double kMaxShooterSpeed = 35;
    public static final double KMinShooterAngle = 3;
    public static final double KMaxShooterAngle = 15.0;

    public static final double MAX_VOLTAGE = 12.0;

    // possibly change per this years gearbox
    // 6380 is top rpm of the falcon, 60 to convert rpm to seconds
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 /
        60.0 *
        SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L3.getWheelDiameter() *
        Math.PI;
    public static final double MAX_MODULE_SPEED = 5.3;
    public static final double DRIVE_ENCODER_COUNTS_PER_REVOLUTION = 2048;
    public static final double DRIVE_ENCODER_CONVERSION_FACTOR = SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L3.getWheelDiameter() *
        Math.PI /
        DRIVE_ENCODER_COUNTS_PER_REVOLUTION;

    private static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.52705;
    private static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705;
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND = 3 * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_PER_SECOND*
    MAX_ANGULAR_VELOCITY_PER_SECOND;

    public static final double DRIVE_BASE_RADIUS = 0.3698875;
    
    //setting up kinematics
    public static final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(
            // Front Left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Front Right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Back Left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Back Right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
        
    // Deadband
    public static final double kDeadband = 0.15;
    public static final double[] XY_Axis_inputBreakpoints =  {-1,  -0.9, -0.85, -0.7, -0.6, -0.5, -0.2,  -0.12, 0.12, 0.2,  0.5, 0.6, 0.7, 0.85, .9, 1};
    public static final double[] XY_Axis_outputTable = {-1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05,  0.2, 0.3, 0.4,  0.6, .7, 1.0};
    public static final double[] RotAxis_inputBreakpoints =  {-1, -.9, -0.85, -0.7, -0.6, -0.5, -0.2,  -0.12, 0.12, 0.2,  0.5, 0.6, 0.7, 0.85, .9, 1};
    public static final double[] RotAxis_outputTable = {-1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05,  0.2, 0.3, 0.4,  0.6, .7, 1.0};
}
