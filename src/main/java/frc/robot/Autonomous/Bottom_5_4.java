package frc.robot.Autonomous;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Enums.IntakeState;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Utilities.PathFollower;

/**
 * Three Note Autonomous Mode.
 * Starts Bottom in front of the speaker and scores the following notes:
 * Initial -> 5 -> 4
 */
public class Bottom_5_4 extends AutonomousMode{

    private PathFollower _pathFollower;
    private Intake _intake;
    private Jukebox _jukebox;
    private Drivetrain _drivetrain;
    private int _count;
    private int _loopCounter;

    public Bottom_5_4()
    {
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        _pathFollower = new PathFollower("Bottom 5-4");
        resetLoopTimer();
    }

    @Override
    public void execute()
    {
        switch(_count)
        {
            case 0:
                _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                nextStep();
                break;
            case 1:
                if (_jukebox.isReadyToScore()){
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 2:
                if (!_jukebox.hasNote())
                {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    // Start Path to Note 5
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();

                }
                break;
            case 3:
                // Follow Path to Note 5
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished())
                {
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 4:
                // Follow Path back to the start
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                }

                if (_pathFollower.isPathFinished())
                {
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 5:
                if (_jukebox.isReadyToScore()) {
                    _jukebox.setState(JukeboxEnum.SCORE);
                    nextStep();
                }
                break;
            case 6:
                if (!_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                    // start bottom to note 4 center
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    nextStep();
                }
                break;
            case 7:
                // Follow Path to Note 4
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                if (_pathFollower.isPathFinished())
                {
                    _pathFollower.startNextPath(new ChassisSpeeds(), _drivetrain.getPose());
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
                break;
            case 8:
                // Follow Path back to Goal
                _drivetrain.drive(_pathFollower.getPathTarget(_drivetrain.getPose()));
                
                if (_jukebox.hasNote()) {
                    _jukebox.setState(JukeboxEnum.PREP_SPEAKER);
                }

                if (_pathFollower.isPathFinished())
                {
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
                if (!_jukebox.hasNote()){
                    _jukebox.setState(JukeboxEnum.IDLE);
                    _drivetrain.drive(new ChassisSpeeds());
                    nextStep();
                }
            // drives to 3
            default:
                _jukebox.setState(JukeboxEnum.IDLE);
                _drivetrain.drive(new ChassisSpeeds());
                break;
        }
        _loopCounter++;
    }

    @Override
    public void abort(){}

    @Override
    public boolean isComplete(){
        return _count >= 13;
    }
    @Override
    public void initialize(){
        var startingPose = _pathFollower.getStartingPose();
        _drivetrain.setStartingPose(startingPose);
        _count = 0;
        _intake.setWantedState(IntakeState.INTAKE);
        _jukebox.setState(JukeboxEnum.IDLE);
        _loopCounter = 0;
    }
    private void nextStep() {
        _count++;
    }
    private void resetLoopTimer()
    {
        _loopCounter = 0;
    }
    private boolean hasElapsed(double seconds)
    {
        // There are 50 loops per second (0.02 seconds)
        // There are 750 loops in 15 seconds
        return ((double)_loopCounter / 50) >= seconds;
    }
}
