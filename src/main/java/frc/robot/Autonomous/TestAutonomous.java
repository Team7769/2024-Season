package frc.robot.Autonomous;

public class TestAutonomous extends AutonomousMode {

    private int _step;
    private int _finalStep;

    public TestAutonomous()
    {
        _step = 0;
        _finalStep = 2;
    }


    public void execute()
    {
        
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
