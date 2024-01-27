package frc.robot.Autonomous;

import java.util.Timer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Time;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.PathFollower;

public class midB23 extends AutonomousMode{
    private PathFollower _pathFollower;    
    private Drivetrain _drivetrain;
    private int _count;
    private Timer _timer = new Timer(); // 15s for auto mode

    public midB23(){
        _drivetrain = Drivetrain.getInstance();
        _pathFollower = new PathFollower("MID B 2 3");
    }

    @Override
    public void execute(){
        switch (_count) {
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
<<<<<<< HEAD
                    nextStep();
=======
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
>>>>>>> 164ab794978da93fe7d5efdce0c6063b5f35823b
                }
                break;
            case 5:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
<<<<<<< HEAD
                    nextStep();
=======
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
>>>>>>> 164ab794978da93fe7d5efdce0c6063b5f35823b
                }
                break;
            case 6:
<<<<<<< HEAD
                _drivetrain.drive(new ChassisSpeeds());
=======
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
>>>>>>> fb686229a7d0314738bd800fd407953e30986610
                break;
        }
    }

    @Override
    public void abort(){}

    @Override
    public boolean isComplete(){
        return _count >= 7;
    }
    @Override
    public void initialize(){
        var startingPose = _pathFollower.getStartingPose();
        _drivetrain.setStartingPose(startingPose);
        _count = 0;
    }
    private void nextStep() {
        _count++;
    }
}
