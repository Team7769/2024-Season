package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.PathFollower;

/**
 * Four Note Autonomous Mode.
 * Starts Mid in front of the speaker and scores the following notes:
 * Initial -> B -> 3 -> 4
 */
public class Mid_B_3_4 extends AutonomousMode{
    private PathFollower _pathFollower;    
    private Drivetrain _drivetrain;
    private Intake _intake;
    private Jukebox _jukebox;
    private int _count;    

    public Mid_B_3_4(){
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        _pathFollower = new PathFollower("MID B-3-4");
    }

    @Override
    public void execute(){
        switch (_count) {
            // preps first shot
            case 0:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
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
            // intakes b
            case 3:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // preps at b
            case 4:
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                    nextStep();
                }
                break;
            // shoots
            case 5:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            // resets and starts path to 3
            case 6:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // drives to 3
            case 7:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // drives back to B
            case 8:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                }
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // scores at b
            case 9:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            // resets after shot
            case 10:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            // drives to 4
            case 11:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            // drives back to B
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
            // shoots at b
            case 13:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 14:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                }
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
        _intake.setWantedState(IntakeState.INTAKE);
        _drivetrain.setStartingPose(startingPose);
        _count = 0;
    }
    private void nextStep() {
        _count++;
    }
}
