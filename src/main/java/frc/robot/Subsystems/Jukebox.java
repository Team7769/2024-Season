package frc.robot.Subsystems;

public class Jukebox {

 enum ElevatorState
 {
    IDK
 }


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

    
}
