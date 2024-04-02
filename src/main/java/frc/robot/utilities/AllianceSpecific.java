package frc.robot.Utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Constants;

public class AllianceSpecific {
    private static boolean isRed() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static Translation2d getSpeaker() {
        return isRed() ? Constants.kRedSpeaker : Constants.kBlueSpeaker;
    }

    public static Rotation2d getAllianceAngleOffset() {
        return Rotation2d.fromDegrees(isRed() ? 180 : 0);
    }
}
