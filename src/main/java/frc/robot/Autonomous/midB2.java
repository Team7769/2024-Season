package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.PathFollower;

public class midB2 extends AutonomousMode {
    
    private PathFollower _pathFollower;
    private Drivetrain _drivetrain;
    private int _step;

    public midB2()
    {
        _drivetrain = Drivetrain.getInstance();
        _pathFollower = new PathFollower("MID SHOOT B SHOOT 2 SHOOT");
    }

    @Override
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
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;

            case 3:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;

            case 4:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;

            case 5:
                _drivetrain.drive(new ChassisSpeeds());
                break;
        }
    }

    @Override
    public void initialize() {
        var startingPose = _pathFollower.getStartingPose();
        _drivetrain.setStartingPose(startingPose);
        _step = 0;
    }

    @Override
    public void abort() {

    }

    @Override
    public boolean isComplete() {
        return _step >= 6;
    }

    private void nextStep() {
        _step++;
    }
}
