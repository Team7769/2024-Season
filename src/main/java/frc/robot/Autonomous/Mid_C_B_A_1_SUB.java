package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.*;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.PathFollower;

/**
 * Five Note Autonomous Mode.
 * Starts Mid in front of the speaker and scores the following notes:
 * Initial -> C -> B -> A -> 3
 */
public class Mid_C_B_A_1_SUB extends AutonomousMode{

    private PathFollower _pathFollower;    
    private Drivetrain _drivetrain;
    private Jukebox _jukebox;
    private Intake _intake;
    private int _count;

    public Mid_C_B_A_1_SUB()
    {
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        _pathFollower = new PathFollower("Mid C-B-A-1 SUB");
    }

    @Override
    public void execute(){
        switch (_count) {
            // preps first shot
            case 0:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER_SUBWOOFER);
                nextStep();
                break;
                // scores first shot
            case 1:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
                // resets robot
            case 2:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // drives to C
            case 3:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // drives back to score C
            case 4:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER_SUBWOOFER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // scores C
            case 5:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            // resets after shot
            case 6:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // drives to B
            case 7:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // drives back to score B
            case 8:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER_SUBWOOFER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // sores B
            case 9:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 10:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // drives to A
            case 11:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // drives back to score A
            case 12:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER_SUBWOOFER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // scores A
            case 13:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 14:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    nextStep();
                }
                break;
                // drives to 1
            case 15:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // drives back to score 1
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
            // scores 1
            case 17:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            // resets after shot
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
        return _count >= 23;
    }
    @Override
    public void initialize(){
        var startingPose = _pathFollower.getStartingPose();
        _intake.setWantedState(IntakeState.INTAKE);
        _drivetrain.setStartingPose(startingPose);
        _count = 0;
    }
    private void nextStep() {
        _count++;
    }
}
