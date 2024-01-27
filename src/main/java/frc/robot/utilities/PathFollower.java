package frc.robot.Utilities;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final PIDConstants _translationConstants = new PIDConstants(1.75,
                                                                        0.0,
                                                                        0.0);

    private final PIDConstants _rotationConstants = new PIDConstants(1.5,
                                                                     0.0,
                                                                     0.0);

    private PPHolonomicDriveController _controller;

    public PathFollower(String autoName) {
        _pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        _startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);

        _timer = new Timer();
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

        _pathIndex += 1;

        // if we have a size of 1 (one path) and an index of 0 marks the first
        // path then on our second run when pathindex reaches 1 it would be
        // out of range of the list
        if (_pathIndex >= _pathGroup.size()) {
            return; // if this is reached, we are doing something wrong
        }

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

    public Pose2d getStartingPose() {
        return _startingPose;
    }

    public boolean isPathFinished() {
        return _timer.hasElapsed(_currentTrajectory.getTotalTimeSeconds());
    }

    public ChassisSpeeds getPathTarget(Pose2d currentPose) {
        State desiredState = _currentTrajectory.sample(_timer.get());

        return _controller.calculateRobotRelativeSpeeds(currentPose,
                                                        desiredState);
    }
}
