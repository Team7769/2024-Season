package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import java.lang.Math;

public class VisionSystem extends Subsystem{

    private static final double filterDistanceError = 2;
    private static final double filterAngleError = 5; 

    private static final double limelightHeight = 0.55;
    //TODO: measure the actual limelightheight

    // private static final double speakerTagBottomHeight = 1.32;

    // private static final double aprilTagHeight = .23;

    private static final double speakerTagHeight = 1.455;
    //height of the middle of the april tag on the speaker


    private static VisionSystem _instance;
    private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * filterDistanceError), 0.02);
	private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2*Math.PI * filterAngleError), 0.02);

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
        //tx = limelightAngleFilter.calculate(tx);
        SmartDashboard.putNumber("VisionSystemGetAngle", tx);
        return tx;  
    }

    public double getDistance()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        var ty = table.getEntry("ty").getDouble(0) * 0.8375; 
        var tx = table.getEntry("tx").getDouble(0); 
        var tv = table.getEntry("tv").getDouble(0);
        
        if (tv != 0.0) {
            //double distance = (speakerTagHeight - limelightHeight) / (Math.tan(Math.toRadians(ty)) * Math.cos(Math.toRadians(tx)));
            //double distance = (speakerTagHeight - limelightHeight) / Math.tan(Math.toRadians(ty));
            double distance = .905 / Math.tan(Math.toRadians(ty));
            SmartDashboard.putNumber("radiansY", Math.toRadians(ty));
            SmartDashboard.putNumber("tanRadiansY", Math.tan(Math.toRadians(ty)));
            double tyRadians = Math.tan(Math.toRadians(ty));
            double txRadians = Math.cos(Math.toRadians(tx));
            double filterDistance = limelightDistanceFilter.calculate(distance);
            SmartDashboard.putNumber("VisionSystemGetDistance", distance);
            return filterDistance; 
        }
        else {
            return 0.0;
        }
    }

    }
