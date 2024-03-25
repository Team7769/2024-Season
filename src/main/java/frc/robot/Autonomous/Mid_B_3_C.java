package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Utilities.PathFollower;

public class Mid_B_3_C extends AutonomousMode {
    private PathFollower _pathFollower;
    private Intake _intake;
    private Jukebox _jukebox;
    private Drivetrain _drivetrain;
    private int _count;

    public Mid_B_3_C(){
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        
        _pathFollower = new PathFollower("Mid B-3-C");
    }

    @Override
    public void execute(){
        switch (_count) {
            case 0:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                nextStep();
                break;
            case 1:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 2:
                if (!_jukebox.hasNote()) {
                   _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            case 3:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }

                break;
            case 4:
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                    nextStep();
                }
                break;
            case 5:
                if (_jukebox.getIsReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 6:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            case 7:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose()); 
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
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
            case 11:
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished()){
                    // At Note A
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 12:
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);                
                    nextStep();
                }
                break;
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
        _jukebox.setIdleSpeedMax();
    }

    private void nextStep() {
        _count++;
    }
}
