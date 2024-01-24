package frc.robot.Utils;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Constants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PathFollower {
    private Timer _timer;
    
    private List<PathPlannerPath> _pathGroup;
    private Pose2d _startingPose;

    private PathPlannerTrajectory _currentTrajectory;

    // init as -1 so that when user starts first path, go to 0
    private int _pathIndex = -1;

    // constants, shall we keep here or move to constants file?
    // private final double _translateKp = 5.5; //5.5
    // private final double _translateKi = 0.0;
    // private final double _translateKd = 0.0;

    // private final double _rotateKp = 4.5; //4.5
    // private final double _rotateKi = 0.0;
    // private final double _rotateKd = 0.0;

    private final PIDConstants _translationConstants = new PIDConstants(5.5,
                                                                        0.0,
                                                                        0.0);

    private final PIDConstants _rotationConstants = new PIDConstants(4.5,
                                                                     0.0,
                                                                     0.0);

    // private PIDController _translationXPID;
    // private PIDController _translationYPID;
    // private PIDController _thetaController;

    private PPHolonomicDriveController _controller;

    PathFollower(String autoName) {
        _pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        _startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        _timer = new Timer();

        // _translationXPID = new PIDController(_translateKp,
        //                                      _translateKi,
        //                                      _translateKd);

        // _translationYPID = new PIDController(_translateKp,
        //                                      _translateKi,
        //                                      _translateKd);

        // _thetaController = new PIDController(_rotateKp, _rotateKi, _rotateKd);
        // _thetaController.enableContinuousInput(-Math.PI, Math.PI);

        _controller = new PPHolonomicDriveController(
            _translationConstants,
            _rotationConstants,
            Constants.MAX_VELOCITY_METERS_PER_SECOND,
            Constants.DRIVE_BASE_RADIUS
        );
    }

    // should be used to start first path
    public void startNextPath(ChassisSpeeds startingSpeeds,
                              Rotation2d startingRotation) {

        if (_pathIndex + 1 >= _pathGroup.size()) {
            return; // if this is reached, we are doing something wrong
        }

        _pathIndex += 1;

        var path = _pathGroup.get(_pathIndex);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            path = path.flipPath();
        }

        _currentTrajectory = path.getTrajectory(startingSpeeds,
                                                startingRotation);


        _timer.reset();
        _timer.start();
    }

    public boolean isPathFinished() {
        return _timer.hasElapsed(_currentTrajectory.getTotalTimeSeconds());
    }

    public ChassisSpeeds getPathTarget(Pose2d currentPose) {
        State desiredState = _currentTrajectory.sample(_timer.get());
        
        // SmartDashboard.putNumber("desiredX", desiredState.poseMeters.getX());
        // SmartDashboard.putNumber("desiredY", desiredState.poseMeters.getY());
        // SmartDashboard.putNumber("desiredZ", desiredState.poseMeters.getRotation().getDegrees());
        // SmartDashboard.putNumber("desiredHolonomic", desiredState.holonomicRotation.getDegrees());

        // PathPlannerServer.sendPathFollowingData(
        // new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        // currentPose);

        return _controller.calculateRobotRelativeSpeeds(currentPose,
                                                        desiredState);
    }
}
