// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Drivetrain;

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

  @Override
  public void robotInit() {
    _drivetrain = Drivetrain.getInstance();
    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    teleopDrive();
  }

  private void teleopDrive() {
    // The X translation will be the vertical value of the left driver joystick
    var translationX = -_driverController.getLeftY();

    // The Y translation will be the horizontal value of the left driver joystick
    var translationY = -_driverController.getLeftX();

    // The rotation will be the horizontal value of the right driver joystick
    var rotation = -_driverController.getRightX();

    _drivetrain.drive(translationX, translationY, rotation);
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
