package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Utilities.PathFollower;

public class Mid_C_B_3_4 extends AutonomousMode {
    private PathFollower _pathFollower;
    private Intake _intake;
    private Jukebox _jukebox;
    private Drivetrain _drivetrain;
    private int _count;

    public Mid_C_B_3_4(){
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        
        _pathFollower = new PathFollower("Mid C-B-3-4");
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

                    // Start Path to Note C (1)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }

                break;
            case 3:
                // Follow Path to Note C (1)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note C (1)
                    _drivetrain.drive(new ChassisSpeeds());
                    //load path to C turn
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }

                break;
            case 4:
                // Follow Path to Note C turn position (1)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note C turn position (1)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 5:
                // Once the note is detected, we can set prep speaker.
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                    nextStep();
                }
                break;
            case 6:
                // Once ready for the shot, set score.
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 7:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note B (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }

                break;
            case 8:
                // Follow Path to Note B (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note B (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 9:
                // Once the note is detected, we can set prep speaker.
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                    nextStep();
                }

                break;
            case 10:            
                    // Once ready for the shot, set score.
                    if (_jukebox.isReadyToScore()) {
                        _jukebox.setState(JukeboxEnum.SCORE);
                        nextStep();
                    }
                break;
            case 11:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note 3 (3)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // Drives to note 3 (3)
            case 12:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // Drives back to Mid
            case 13:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // Scores at Mid
            case 14:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 15:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    nextStep();
                }
                break;
            // Drives to Note 4
            case 16:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // DONE
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
