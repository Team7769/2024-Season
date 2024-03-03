package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Utilities.PathFollower;

public class Mid_B_A_1_2 extends AutonomousMode {
    private PathFollower _pathFollower;
    private Intake _intake;
    private Jukebox _jukebox;
    private Drivetrain _drivetrain;
    private int _count;

    public Mid_B_A_1_2(){
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        
        _pathFollower = new PathFollower("Mid B-A-1-2");
    }

    @Override
    public void execute(){
        switch (_count) {
            case 0:
                // Initial Position - Prep for Speaker Shot
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                nextStep();
                break;
            case 1:
                if (_jukebox.isReadyToScore()) {
                    // Once ready to score, set score state.
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 2:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                   _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note B (1)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }

                break;
            case 3:
                // Follow Path to Note B (1)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note B (1)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 4:
                // Once the note is detected, we can set prep speaker.
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                    nextStep();
                }
                break;
            case 5:
                // Once ready for the shot, set score.
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 6:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note A (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }

                break;
            case 7:
                // Follow Path to Note A (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note A (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 8:
                // Once the note is detected, we can set prep speaker.
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                    nextStep();
                }

                break;
            case 9:            
                    // Once ready for the shot, set score.
                    if (_jukebox.isReadyToScore()) {
                        _jukebox.setState(JukeboxEnum.SCORE);
                        nextStep();
                    }
                break;
            case 10:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note 1
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // Drives to note 1
            case 11:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // Drives back to A
            case 12:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // Scores at A
            case 13:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 14:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    nextStep();
                }
                break;
            case 15:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // Drives back to A
            case 16:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // Scores at A
            case 17:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 18:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    nextStep();
                }
                break;
            default:
                _drivetrain.drive(new ChassisSpeeds());
                break;
            }
        }

    @Override
    public void abort(){}

    @Override
    public boolean isComplete(){
        return _count >= 15;
    }
    @Override
    public void initialize(){
        var startingPose = _pathFollower.getStartingPose();
        _drivetrain.setStartingPose(startingPose);
        _count = 0;

        // Set initial subsystem states. We should never need to change the Intake state as it is autonomous.
        _intake.setWantedState(IntakeState.INTAKE);
        _jukebox.setState(JukeboxEnum.IDLE);
    }

    private void nextStep() {
        _count++;
    }
}
