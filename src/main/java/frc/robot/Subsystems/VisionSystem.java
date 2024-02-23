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

    private double _targetDistance = 0.0;
    private double _targetAngle = 0.0; // dont need


    private static VisionSystem _instance;
    private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * filterDistanceError), 0.02);
	private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2*Math.PI * filterAngleError), 0.02);

    VisionSystem() {}

    public static VisionSystem getInstance() {
        if (_instance == null) {
            _instance = new VisionSystem();
        }

        return _instance;
    }

    public double getTargetAngle() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        double validTargets = table.getEntry("tv").getDouble(0);

        // read targetangle regardless for smartdashboard data
        double targetAngle = table.getEntry("tx").getDouble(0);

        SmartDashboard.putNumber("VisionSystemGetAngle", targetAngle);

        if (validTargets > 0) {
            _targetAngle = targetAngle;
        } else {
            // very hacky way 
            return 7769.0;
        }
 
        //tx = limelightAngleFilter.calculate(tx);

        return _targetAngle;  
    }

    public double getDistance() {
        NetworkTable table = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

        // what is ty?
        var ty = table.getEntry("ty").getDouble(0) * 1.2; // constant

        double validTargets = table.getEntry("tv").getDouble(0);
        
        if (validTargets > 0) {
            double distance = .905 / Math.tan(Math.toRadians(ty)); // constant

            double filteredDistance = limelightDistanceFilter.calculate(
                distance
            );

            SmartDashboard.putNumber("VisionSystemGetDistance", distance);

            _targetDistance = filteredDistance;
        }

        return _targetDistance;
    }

    public double[] getTargetingInfo() {
        NetworkTable table = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

        double validTargets = table.getEntry("tv").getDouble(0);

        double ty = table.getEntry("ty").getDouble(0.0) * 1.2; // constant

        // read targetangle regardless for smartdashboard data
        double targetAngle = table.getEntry("tx").getDouble(0.0);

        double targetDistance = .905 / Math.tan(Math.toRadians(ty));

        SmartDashboard.putNumber("VisionSystemGetAngle", targetAngle);
        SmartDashboard.putNumber("VisionSystemGetDistance", targetDistance);

        if (validTargets > 0.0) {
            double filteredDistance = limelightDistanceFilter.calculate(
                targetDistance
            );

            _targetDistance = filteredDistance;
        }

        double[] returnArray = {validTargets, _targetDistance, targetAngle};

        return returnArray;
    }
}
