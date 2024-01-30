package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSystem {

    private static VisionSystem _instance;

    VisionSystem () {

    }

    public static VisionSystem getInstance() {
        if (_instance == null) {
            _instance = new VisionSystem();
        }

        return _instance;
    }

    // Example metod for getting the botpose from the limelight
    public Pose2d getBotpose() {
        // Normal botpose array

        var tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1.0);

        if (tid < 0) {
            return null;
        }
        
        // Alliance specific botpose array
        double[] wpiBotposeArray;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            wpiBotposeArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
        } else {
            wpiBotposeArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }

        // Example: Return new Pose with the three array values.
        return new Pose2d(wpiBotposeArray[0], wpiBotposeArray[1], Rotation2d.fromDegrees(wpiBotposeArray[5]));
    }

    public double getLatency() {
        double[] wpiBotposeArray;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            wpiBotposeArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
        } else {
            wpiBotposeArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }

        return wpiBotposeArray[6];
    }
}
