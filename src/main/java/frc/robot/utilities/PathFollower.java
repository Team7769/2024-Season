package frc.robot.Utilities;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Constants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private final Field2d _pathField = new Field2d();
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

        SmartDashboard.putData("Path Field", _pathField);
    }

    // should be used to start first path
    public void startNextPath(ChassisSpeeds startingSpeeds,
                              Rotation2d startingRotation) {

        if (_pathIndex + 1 > _pathGroup.size()) {
            return; // if this is reached, we are doing something wrong
        }

        _pathIndex += 1;

        SmartDashboard.putNumber("pathIndex", _pathIndex);
        var path = _pathGroup.get(_pathIndex);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            path = path.flipPath();
            //startingRotation = GeometryUtil.flipFieldRotation(startingRotation);
        }

        _currentTrajectory = path.getTrajectory(startingSpeeds,
                                                startingRotation);

        _pathField.setRobotPose(_currentTrajectory.getInitialTargetHolonomicPose());
        _timer.reset();
        _timer.start();
    }

    public Pose2d getStartingPose() {
        var alliance = DriverStation.getAlliance();
        var shouldFlipPath = alliance.isPresent() && alliance.get() == Alliance.Red;

        var startingPose = shouldFlipPath ? GeometryUtil.flipFieldPose(_startingPose) : _startingPose;

        return startingPose;
    }

    public boolean isPathFinished() {
        return _timer.hasElapsed(_currentTrajectory.getTotalTimeSeconds());
    }

    public ChassisSpeeds getPathTarget(Pose2d currentPose) {
        State desiredState = _currentTrajectory.sample(_timer.get());
        _pathField.setRobotPose(desiredState.getTargetHolonomicPose());

        return _controller.calculateRobotRelativeSpeeds(currentPose,
                                                        desiredState);
    }
}
