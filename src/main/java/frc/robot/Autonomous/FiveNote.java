package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Utilities.PathFollower;

public class FiveNote extends AutonomousMode{
    private PathFollower _pathFollower;
    private Intake _intake;
    private Jukebox _jukebox;
    private Drivetrain _drivetrain;
    private int _count;    

    public FiveNote(){
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        
        _pathFollower = new PathFollower("5 Note");
    }

    @Override
    public void execute(){
        switch (_count) {
            case 0:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                // Start Path to Note C (1)
                _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                nextStep();
                break;
            case 1:
                // Follow Path to Note C (1)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note C (1)
                    _drivetrain.drive(new ChassisSpeeds());

                    // Start Path to Note B (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;

            case 2:
                // Follow Path to Note B (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note B (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    
                    // Start Path to Note A (3)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;

            case 3:
                // Follow Path to Note A (3)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note B (3)
                    _drivetrain.drive(new ChassisSpeeds());
                    
                    // Start Path to Note 1 (4)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;
            case 4:
                // Follow Path to Note 1 (4)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note 1 (4)
                    _drivetrain.drive(new ChassisSpeeds());
                    
                    // Start Path back to Note A (4)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }
                break;

            case 5:
                // Follow Path to Note A (4)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    // At Note A (4)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 6:
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

        _intake.setWantedState(IntakeState.INTAKE);
        _jukebox.setState(JukeboxEnum.IDLE);
    }
    private void nextStep() {
        _count++;
    }
}
