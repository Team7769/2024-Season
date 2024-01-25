package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.PathFollower;

public class TestAutonomous extends AutonomousMode {

    private int _step;
    private int _finalStep;
    private PathFollower _pathFollower;
    private Drivetrain _drivetrain;

    public TestAutonomous()
    {
        _step = 0;
        _finalStep = 2;

        _drivetrain = Drivetrain.getInstance();
        _pathFollower = new PathFollower("Test");
    }


    public void execute()
    {
        switch (_step) {
            case 0:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;
            case 1:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 2:
                _drivetrain.drive(new ChassisSpeeds());
                break;
        }
    }

    public void initialize()
    {
        _pathFollower.startNextPath(new ChassisSpeeds(), new Rotation2d());
        var startingPose = _pathFollower.getStartingPose();
        _drivetrain.setStartingPose(startingPose, startingPose.getRotation().getDegrees());
    }

    public void abort()
    {

    }

    public boolean isComplete()
    {
        return _step > _finalStep;
    }

    private void nextStep() {
        _step++;
    }
}
