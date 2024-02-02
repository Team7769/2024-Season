package frc.robot.Subsystems;

public class Jukebox {

    private ElevatorState currentState;




    private static Jukebox _instance;

    public Jukebox()
    {
        
    }

    public static Jukebox getInstance()
    {
        if (_instance == null)
        {
            _instance = new Jukebox();
        }

        return _instance;
    }

    // @Override
    // public void logTelemetry(){}

    private void switchCurrentState(){}

    public void setState(ElevatorState e)
    {
        switch (e) {
            case IDK:
                
                break;
        
            case RAMPUP:
                break;
            case SHOOT:
                break;
            case RESET:
                break;
            case DUMPAMP:
                break;
            case SETUPFORAMP:
                break;
            case EXTEND:
                break;
        }
    }

    
}
