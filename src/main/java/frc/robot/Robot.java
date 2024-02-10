// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Autonomous.AutonomousMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Constants;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.VisionSystem;
import frc.robot.Utilities.AutoUtil;
import frc.robot.Utilities.OneDimensionalLookup;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   private Drivetrain _drivetrain;
   private Intake _intake;
   private XboxController _driverController;
   private XboxController _operatorController;
   private SendableChooser<Integer> _autoChooser = new SendableChooser<>();
   private AutonomousMode _currentAuto;
   private VisionSystem _visionSystem;
   private Jukebox _jukebox;

  @Override
  public void robotInit() {
    _drivetrain = Drivetrain.getInstance();
    _jukebox = Jukebox.getInstance();
    _intake = Intake.getInstance();

    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);

    _visionSystem = VisionSystem.getInstance();
    // loads the auto modes
    AutoUtil.autonmousDropDown(_autoChooser);
    // puts the drop down to select auton modes on shuffleboard
    SmartDashboard.putData("Selected Auto Mode", _autoChooser);
    // finds the selected autonomous
  }

  @Override
  public void robotPeriodic() {
    _drivetrain.logTelemetry();
    _jukebox.logTelemetry();
    _intake.logTelemetry();
  }

  @Override
  public void autonomousInit() {
    _drivetrain.reset();
    _currentAuto = AutoUtil.selectedAuto(_autoChooser.getSelected());
    _currentAuto.initialize();
  }

  @Override
  public void autonomousPeriodic() {
    _drivetrain.updateOdometry();
    _currentAuto.execute();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    teleopDrive();
    _drivetrain.updateOdometry();
  }

  private void teleopDrive() {
    // The X translation will be the vertical value of the left driver joystick
    var translationX = -OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints,
        Constants.XY_Axis_outputTable, _driverController.getLeftY());

    // The Y translation will be the horizontal value of the left driver joystick
    var translationY = -OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints,
        Constants.XY_Axis_outputTable, _driverController.getLeftX());

    // The rotation will be the horizontal value of the right driver joystick
    var rotation = -OneDimensionalLookup.interpLinear(Constants.RotAxis_inputBreakpoints,
        Constants.RotAxis_outputTable,
        _driverController.getRightX());

    if (_driverController.getRightBumper())
    {
        rotation = _visionSystem.getTargetAngle() / 27 ;
        //target angle range is -27 to 27 degrees
    }
    

    if (_driverController.getBackButton() && _driverController.getStartButton())
    {
      _drivetrain.reset();
    }

    _drivetrain.fieldOrientedDrive(translationX, translationY, rotation);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    _intake.setWantedState(IntakeState.STOP);
    _jukebox.resetSensors();
  }

  @Override
  public void testPeriodic() {
    testOperate();

    _intake.handleCurrentState();
    _jukebox.handleCurrentState();
  }

  private void testOperate() {   
    _jukebox.setState(JukeboxEnum.MANUAL);

    if (_operatorController.getLeftBumper())
    {
      _jukebox.setManualFeederSpeed(.5);
    } else if (_operatorController.getRightBumper())
    {
      _jukebox.setManualFeederSpeed(-.5);
    } else 
    {
      _jukebox.setManualFeederSpeed(0);
    }

    if (_operatorController.getAButton()) {
      _intake.setWantedState(IntakeState.INTAKE);
    } else if (_operatorController.getBButton()) {
      _intake.setWantedState(IntakeState.STOP);
    } else if (_operatorController.getXButton()) {
      _intake.setWantedState(IntakeState.EJECT);
    } else if (_operatorController.getYButton()) {
      _intake.setWantedState(IntakeState.PASSIVE_EJECT);
    }

    if (_driverController.getYButton()) {
      _jukebox.setManualElevatorSpeed(1);
    } else if (_driverController.getRightBumper()) {
      _jukebox.setManualElevatorSpeed(-.5);
    } else {
      _jukebox.setManualElevatorSpeed(0.0);
    }

    if (_driverController.getXButton()) {
      _jukebox.setManualShooterAngleSpeed(-0.5);
    } else if (_driverController.getBButton()) {
      _jukebox.setManualShooterAngleSpeed(0.5);
    } else {
      _jukebox.setManualShooterAngleSpeed(0.0);
    }
    
    if (_driverController.getAButton()) {
      _jukebox.setManualShooterSpeed(1.0);
    } else {
      _jukebox.setManualShooterSpeed(0.0);
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}


}
