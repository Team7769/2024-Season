package frc.robot.Autonomous;

import com.pathplanner.lib.auto.AutoBuilder;

public abstract class AutonomousMode 
{

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
        return false;
    }

    public static void configAuton()
    {
        AutoBuilder.configureHolonomic();
    }
}