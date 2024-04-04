package frc.robot.Utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Constants;

public class AllianceSpecific {
    private static boolean isRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static Translation2d getSpeaker() {
        return isRed() ? Constants.kRedSpeaker : Constants.kBlueSpeaker;
    }

    public static Translation2d getZone() {
        return isRed() ? Constants.kRedZone : Constants.kBlueZone;
    }

    public static Rotation2d getAllianceAngleOffset() {
        //return Rotation2d.fromDegrees(isRed() ? 180 : 0);
        return Rotation2d.fromDegrees(180);
    }

    public static Translation2d mirrorTranslation(
        Translation2d blueTranslation
    ) {
        return new Translation2d(
            blueTranslation.getX() +
                Constants.kFieldLength -
                (2 * blueTranslation.getX()),

            blueTranslation.getY()
        );
    }
}
