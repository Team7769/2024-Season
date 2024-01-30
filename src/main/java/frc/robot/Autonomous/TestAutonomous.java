package frc.robot.Autonomous;

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
                _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                nextStep();
                break;
            case 1:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;
            case 2:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 3:
                _drivetrain.drive(new ChassisSpeeds());
                break;
        }
    }

    public void initialize()
    {        
        var startingPose = _pathFollower.getStartingPose();
        _drivetrain.setStartingPose(startingPose);
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
