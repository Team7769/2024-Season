package frc.robot.Subsystems;

import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Constants;

public class Drivetrain {
    private static Drivetrain _instance;

    private SwerveModule _frontLeftModule;
    private SwerveModule _frontRightModule;
    private SwerveModule _backLeftModule;
    private SwerveModule _backRightModule;

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
    }

    public static Drivetrain getInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }

        return _instance;
    }

    public void drive(double translationX, double translationY, double rotationZ)
    {

    }
}
