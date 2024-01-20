package frc.robot.Autonomous;

public class TestAutonomous extends AutonomousMode
{
    private int _step;
    private int _finalStep;

    public TestAutonomous(){
        _step = 0;
        _finalStep = 2;
    }

    /**
     * Executes the tasks for the autnomous mode.
     */
    @Override
    public void execute()
    {
        switch (_step){
            case(0):
                
                _step++;
                break;
            case(1):
                
                _step++;
                break;
            case(2):
                _step++;
                break;    
        }
    }

    @Override
    public void initialize()
    {
        
    }

    /** 
     * Stops the autnomous mode.
    */
    @Override
    public void abort()
    {

    }

    /**
     * Returns true if the step count is above the final step number.
     */
    @Override
    public boolean isComplete()
    {
        return (_step > _finalStep);
    }
}
