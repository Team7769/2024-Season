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
        _pathFollower = new PathFollower("MID SHOOT B SHOOT 2 SHOOT");
    }

    @Override
    public void execute(){
        switch (_count) {
            case 0:
                _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                _count++;
                break;

            case 1:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
                }
                break;

            case 2:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
                }
                break;

            case 3:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
                }
                break;

            case 4:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _count++;
                }
                break;

            case 5:
                _drivetrain.drive(new ChassisSpeeds());
                break;
            case 6:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _count++;
                }
                break;
            default:
                System.out.println("Program fail.");
                _drivetrain.drive(new ChassisSpeeds());
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
}
