// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Autonomous.AutonomousMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Constants;
import frc.robot.Enums.*;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.AutoUtil;
import frc.robot.Utilities.LEDController;
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
   private LEDController _ledController;
   private boolean _score;
   private boolean _scoreReleased;

  @Override
  public void robotInit() {
    _drivetrain = Drivetrain.getInstance();
    _jukebox = Jukebox.getInstance();
    _intake = Intake.getInstance();
    _ledController = LEDController.getInstance();

    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);
    _visionSystem = VisionSystem.getInstance();
    // loads the auto modes
    AutoUtil.autonmousDropDown(_autoChooser);
    // puts the drop down to select auton modes on shuffleboard
    SmartDashboard.putData("Selected Auto Mode", _autoChooser);
    // finds the selected autonomous
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    _drivetrain.logTelemetry();
    _jukebox.logTelemetry();
    _intake.logTelemetry();
    _ledController.handleLights();;
  }

  @Override
  public void autonomousInit() {
    _drivetrain.reset();
    _jukebox.resetSensors();
    _currentAuto = AutoUtil.selectedAuto(_autoChooser.getSelected());
    _currentAuto.initialize();
    _jukebox.enableAutoSpinup();
  }

  @Override
  public void autonomousPeriodic() {
    //_ledController.handleLights();
    // _ledController.handleBottomLights(); Add this code when we get the bottom lights setup on the robot
    _drivetrain.updateOdometry();
    _currentAuto.execute();
    _intake.handleCurrentState();
    _jukebox.handleCurrentState();
  }

  @Override
  public void teleopInit() {
    _intake.setWantedState(IntakeState.INTAKE);
    _jukebox.disableAutoSpinup();
  }

  @Override
  public void teleopPeriodic() {
    //_ledController.handleLights();
    // _ledController.handleBottomLights(); Add this code when we get the bottom lights setup on the robot
    teleopDrive();
    teleopJukebox();
    _drivetrain.updateOdometry();

    teleopIntake();
    teleopClimb();
    _intake.handleCurrentState();
    _jukebox.handleCurrentState();
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

    _score = Math.abs(_driverController.getRightTriggerAxis()) > 0.25;

    if (_driverController.getBButton() && _driverController.getAButton()) {
      _drivetrain.reset();
    }

    if (Math.abs(_driverController.getLeftTriggerAxis()) > 0.25)
    {
        rotation = -(_visionSystem.getTargetAngle() / 105) ;
        //target angle range is -27 to 27 degrees
    }

    // if (_driverController.getBackButton() && _driverController.getStartButton())
    // {
    //   _drivetrain.reset();
    // }

    _drivetrain.fieldOrientedDrive(translationX, translationY, rotation);
  }

  public void teleopJukebox() {
<<<<<<< HEAD
        if (Math.abs(_operatorController.getLeftTriggerAxis()) > 0.25) {
          if (_jukebox.hasNote())
            _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
        }  else if (Math.abs(_operatorController.getRightTriggerAxis()) > 0.25) {
          if (_jukebox.hasNote())
            _jukebox.setState(JukeboxEnum.PREP_AMP);
        } else if (_operatorController.getXButton()) {
          if (_jukebox.hasNote())
            _jukebox.setState(JukeboxEnum.PREP_SPEAKER_LINE);
        } else if (_operatorController.getYButton()) {
          if (_jukebox.hasNote())
            _jukebox.setState(JukeboxEnum.PREP_SPEAKER_PODIUM);
        } else if (_operatorController.getBButton()) {
          if (_jukebox.hasNote())
            _jukebox.setState(JukeboxEnum.PREP_TRAP);
        } else if (_operatorController.getAButton()) {
          _jukebox.setState(JukeboxEnum.PREP_HUMAN_INTAKE);
        } else if (_operatorController.getLeftBumper()) {
          _jukebox.setState(JukeboxEnum.PREP_LAUNCH);
        }

        if (_driverController.getXButton()) {
          _jukebox.setState(JukeboxEnum.JUKEBOX_TEST);
        }

        if (_score) {
          _jukebox.setState(JukeboxEnum.SCORE);
        } else if (_scoreReleased) {
          if (_jukebox.getPreviousState() != JukeboxEnum.PREP_TRAP) {
            _jukebox.setState(JukeboxEnum.IDLE);
          }
        }
        _scoreReleased = _score;

        if (_operatorController.getBackButton()) {
          _jukebox.setState(JukeboxEnum.IDLE);
        }
      }
=======
  if (_jukebox.hasNote()) {
    if (Math.abs(_operatorController.getLeftTriggerAxis()) > 0.25) {
      _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
    }  else if (Math.abs(_operatorController.getRightTriggerAxis()) > 0.25) {
      _jukebox.setState(JukeboxEnum.PREP_AMP);
      //_jukebox.setState(JukeboxEnum.PREP_SPEAKER_LINE);
    } else if (_operatorController.getXButton()) {
      _jukebox.setState(JukeboxEnum.PREP_SPEAKER_LINE);
      //_jukebox.setState(JukeboxEnum.PREP_AMP);
    } else if (_operatorController.getYButton()) {
      _jukebox.setState(JukeboxEnum.PREP_SPEAKER_PODIUM);
    } else if (_operatorController.getBButton()) {
      _jukebox.setState(JukeboxEnum.PREP_TRAP);
    } else if (_operatorController.getLeftBumper()) {
      _jukebox.setState(JukeboxEnum.PREP_LAUNCH);
    }
  }
  if (_driverController.getXButtonPressed()) {
    // _jukebox.setState(JukeboxEnum.JUKEBOX_TEST);
    if (_jukebox.getDisableAutoSpinup()) {
      _jukebox.enableAutoSpinup();
    } else {
      _jukebox.disableAutoSpinup();
    }
  }

  if (_score) {
    _jukebox.setState(JukeboxEnum.SCORE);
  } else if (_scoreReleased) {
    if (_jukebox.getPreviousState() == JukeboxEnum.PREP_TRAP) {
      _jukebox.setState(JukeboxEnum.PREP_TRAP);
    } else {
      _jukebox.setState(JukeboxEnum.IDLE);
    }
  }
  _scoreReleased = _score;

  if (_operatorController.getBackButton()) {
    _jukebox.setState(JukeboxEnum.IDLE);
  }
}
>>>>>>> 120bf681f41c0ad1418646b1cc2d0070a191f300

  private void teleopIntake() {
    if (_operatorController.getStartButton()) {
      // emergency eject
      _jukebox.setState(JukeboxEnum.EJECT);
      _intake.setWantedState(IntakeState.EJECT);
    } else if (_operatorController.getStartButtonReleased()) {
      // passive_eject is a default state and will automatically change to
      // intake if a note isnt held
      _jukebox.setState(JukeboxEnum.IDLE);
      _intake.setWantedState(IntakeState.PASSIVE_EJECT);
    }
  }

  private void teleopClimb() {
        // if(_driverController.getYButtonPressed()) {
    //   JukeboxEnum wantedState = _jukebox.getState() == JukeboxEnum.EXTEND_FOR_CLIMB ? 
    //                             JukeboxEnum.CLIMB : JukeboxEnum.EXTEND_FOR_CLIMB;

    //   _jukebox.setState(wantedState);
    // }

    if (_driverController.getBackButtonPressed()) {
      _jukebox.setState(JukeboxEnum.EXTEND_FOR_CLIMB);

    }
    if (_driverController.getStartButtonPressed()) {
      _jukebox.setState(JukeboxEnum.CLIMB);
    } else if (_driverController.getStartButtonReleased()) {
      if (_jukebox.getElevatorPosition() > 1) {
        _jukebox.setState(JukeboxEnum.EXTEND_FOR_CLIMB);
      }
    }

    if (_operatorController.getBButton()) {
      _jukebox.setState(JukeboxEnum.PREP_TRAP);
    }
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
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }


}
