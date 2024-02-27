package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Drivetrain;

public class DoNothing extends AutonomousMode {

    private int _step;
    private int _finalStep;
    private Drivetrain _drivetrain;

    /**Actually does nothing. */
    public DoNothing()
    {
        _step = 0;
        _finalStep = 2;

        _drivetrain = Drivetrain.getInstance();
    }


    public void execute()
    {
        switch (_step) {
            default:
                _drivetrain.drive(new ChassisSpeeds());
                break;
        }
    }

    public void initialize()
    {        
    }

    public void abort()
    {

    }

    public boolean isComplete()
    {
        return _step > _finalStep;
    }
}
