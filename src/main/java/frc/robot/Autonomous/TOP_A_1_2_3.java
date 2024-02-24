package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Utilities.PathFollower;

public class TOP_A_1_2_3 extends AutonomousMode {
    private int _step = 0;

    private Drivetrain _drivetrain;
    private Intake _intake;
    private Jukebox _jukebox;

    private PathFollower _pathFollower;

    public TOP_A_1_2_3() {
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();

        _pathFollower = new PathFollower("TOP_A_1_2_3");
    }

    @Override
    public void execute() {
        switch (_step) {
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

                    // Start Path to Note A (1)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }

                break;
            case 3:
                // Follow Path to Note A (1)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note A (1)
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

                    // Start Path to Note 1 (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }

                break;
            case 7:
                // Follow Path to Note 1 (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note 1 (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 8:
                // Once the note is detected, we can go back.
                if (_jukebox.hasNote()) {
                    // Start Path to Note A (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }

                break;
            case 9:            
                // Follow Path to Note A (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note A (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 10:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                nextStep();

                break;
            case 11:
                // Once ready for the shot, set score.
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 12:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note 1 (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }

                break;
            case 13:
                // Follow Path to Note 2 (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note 1 (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 14:
                // Once the note is detected, we can go back.
                if (_jukebox.hasNote()) {
                    // Start Path to Note A (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }

                break;
            case 15:            
                // Follow Path to Note A (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note A (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 16:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                nextStep();

                break;
            case 17:
                // Once ready for the shot, set score.
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 18:
                // After the note has left the robot, transition to idle and start the next path.
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);

                    // Start Path to Note 3 (2)
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getGyroRotationWithOffset());
                    nextStep();
                }

                break;
            case 19:
                // Follow Path to Note 3 (2)
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));

                if (_pathFollower.isPathFinished()){
                    // At Note 3 (2)
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            default:
                _drivetrain.drive(new ChassisSpeeds());
                break;
        }
    }

    private void nextStep() {
        _step++;
    }

    @Override
    public boolean isComplete(){
        return _step >= 20;
    }

    @Override
    public void initialize() {
        var startingPose = _pathFollower.getStartingPose();

        _intake.setWantedState(IntakeState.INTAKE);

        _drivetrain.setStartingPose(startingPose);

        _step = 0;
    }
}
