package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.PathFollower;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;

public class midB23 extends AutonomousMode{
    private PathFollower _pathFollower;    
    private Drivetrain _drivetrain;
    private Intake _intake;
    private Jukebox _jukebox;
    private int _count;
    public midB23(){
        _drivetrain = Drivetrain.getInstance();
        _pathFollower = new PathFollower("MID B 2 3");
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
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
            //shoot
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                //need to set state to score somehow
                if(!_jukebox.hasNote())
                {
                    nextStep(); 
                }
                
            case 3:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;
            case 4:
                _intake.setWantedState(IntakeState.INTAKE);
                if(_jukebox.hasNote())
                {
                    _intake.setWantedState(IntakeState.PASSIVE_EJECT);
                    nextStep();
                }
            case 5:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                    if (_pathFollower.isPathFinished()){
                        _drivetrain.drive(new ChassisSpeeds());
                        nextStep();
                        _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                        _count++;
                    }
                    break;
            case 6: 
                    //shoot
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                //need to set state to score somehow
                if(!_jukebox.hasNote())
                {
                    nextStep(); 
                }
            case 7: 
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                        if (_pathFollower.isPathFinished()){
                            _drivetrain.drive(new ChassisSpeeds());
                            nextStep();
                            _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                            _count++;
                        }
                        break;
            case 8:
            _intake.setWantedState(IntakeState.INTAKE);
                if(_jukebox.hasNote())
                {
                    _intake.setWantedState(IntakeState.PASSIVE_EJECT);
                    nextStep();
                }
            case 9: 
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
                }
                break;
            case 10:
                    //shoot
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                    //need to set state to score somehow
                    if(!_jukebox.hasNote())
                    {
                        nextStep(); 
                    }
            case 11:
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            }
            

            

            
                
            

            /*
            case 5:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    _count++;
                }
                break;
            case 6:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
        }*/
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
