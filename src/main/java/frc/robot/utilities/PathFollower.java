package frc.robot.Utilities;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Constants;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class to follow a path based on provided information. Used to move to desired locations in Autonomous.
 */
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

    // This Field2d will display the path targets as a ghost
    private final Field2d _pathField = new Field2d();
    private boolean _shouldFlipPath = false;
    
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

    /**Starts the next path in the auto sequence.
     * @param startingSpeeds - The chassis speeds when starting this path
     * @param currentPose - The current pose of the robot. This must already be transformed to the proper alliance.
     */
    public void startNextPath(ChassisSpeeds startingSpeeds,
                              Pose2d currentPose) {

        if (_pathIndex + 1 > _pathGroup.size()) {
            return; // if this is reached, we are doing something wrong
        }

        _pathIndex += 1;

        _controller.reset(currentPose, new ChassisSpeeds());
        SmartDashboard.putNumber("pathIndex", _pathIndex);
        var path = _pathGroup.get(_pathIndex);

        if (_shouldFlipPath) {
            path = path.flipPath();
        }

        _currentTrajectory = path.getTrajectory(startingSpeeds,
                                                currentPose.getRotation());
        PPLibTelemetry.setCurrentPath(path);

        _pathField.setRobotPose(_currentTrajectory.getInitialTargetHolonomicPose());
        _timer.reset();
        _timer.start();
    }

    /**
     * Returns the starting pose of the robot loaded from the auto file. It will automatically handle transformation based on alliance.
     * @return The starting pose of the robot.
     */
    public Pose2d getStartingPose() {
        var alliance = DriverStation.getAlliance();
        _shouldFlipPath = alliance.isPresent() && alliance.get() == Alliance.Red;

        var startingPose = _shouldFlipPath ? GeometryUtil.flipFieldPose(_startingPose) : _startingPose;

        PPLibTelemetry.setCurrentPose(startingPose);
        return startingPose;
    }

    /**
     * Indicates if the path has finished running. 
     * @return If the path has exceeded its running time.
     */
    public boolean isPathFinished() {
        return _timer.hasElapsed(_currentTrajectory.getTotalTimeSeconds());
    }

    /**
     * Calculates and returns the chassis speeds to provided to the drivetrain that will follow the path.
     * @param currentPose The current pose of the robot.
     * @return The desired chassis speeds to follow the path.
     */
    public ChassisSpeeds getPathTarget(Pose2d currentPose) {
        Rotation2d rotation;
        if (_shouldFlipPath) {
            rotation = GeometryUtil.flipFieldRotation(currentPose.getRotation());
            SmartDashboard.putNumber("flippedFieldRotation", rotation.getDegrees());
        }

        State desiredState = _currentTrajectory.sample(_timer.get());
        _pathField.setRobotPose(desiredState.getTargetHolonomicPose());

        PPLibTelemetry.setCurrentPose(currentPose);
        PPLibTelemetry.setTargetPose(desiredState.getTargetHolonomicPose());

        return _controller.calculateRobotRelativeSpeeds(currentPose,
                                                        desiredState);
    }
}
