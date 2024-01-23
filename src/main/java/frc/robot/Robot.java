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
import frc.robot.Subsystems.Drivetrain;
import frc.robot.utilities.AutoUtil;
import frc.robot.utilities.OneDimensionalLookup;
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
   private XboxController _driverController;
   private XboxController _operatorController;
   private SendableChooser<Integer> _autoChooser = new SendableChooser<>();
   private AutonomousMode _currentAuto;

  @Override
  public void robotInit() {
    _drivetrain = Drivetrain.getInstance();
    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);

    _autoChooser.setDefaultOption("Do Nothing", 0);
    _autoChooser.addOption("TestAutnomous", 1);

    SmartDashboard.putData(_autoChooser);
    SmartDashboard.putData("Selected Auto Mode", _autoChooser);
    _currentAuto = AutoUtil.selectedAuto(_autoChooser.getSelected());
  }

  @Override
  public void robotPeriodic() {
    _drivetrain.logTelemetry();
    _drivetrain.updateOdometry();
  }

  @Override
  public void autonomousInit() {
    _currentAuto.initialize();
  }

  @Override
  public void autonomousPeriodic() {
    _currentAuto.execute();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    teleopDrive();
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
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
