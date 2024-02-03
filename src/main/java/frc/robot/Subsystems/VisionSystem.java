package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSystem extends Subsystem{

    private static VisionSystem _instance;

    VisionSystem()
    {

    }

    public static VisionSystem getInstance() {
        if (_instance == null) {
            _instance = new VisionSystem();
        }

        return _instance;
    }

    public double getTargetAngle()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        var tx = table.getEntry("tx").getDouble(0);  
        return tx;  
    }



    }
