package frc.robot.Subsystems;

public class Drivetrain {
    
    private static Drivetrain _instance;
    private Drivetrain()
    {

    }

    public static Drivetrain getInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }

        return _instance;
    }
}
