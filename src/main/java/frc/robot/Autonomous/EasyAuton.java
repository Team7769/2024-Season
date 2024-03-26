package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Subsystems.VisionSystem;
import frc.robot.Utilities.PathFollower;

public class EasyAuton {
    // public static int FOLLOW_WITH_POSE_AIM = 0;
    // public static int FOLLOW_WITH_SHOOT_AIM = 1;
    // public static int FOLLOW_WITH_FULL_AIM = 2;
    // public static int FOLLOW_WITH_PASSTHROUGH = 3;
    // public static int FOLLOW = 4;
    // public static int SHOOT = 5;
    // public static int AIM = 6;
    // public static int POSE_AIM = 7;
    // public static int SHOOT_AIM = 8;

    private static final String[] _internalKeys = {};
    private static final int[][] _internalAutons = {};

    private AutoCmds[] _auton;

    private int _step = 0;

    private Drivetrain _drivetrain;
    private Intake _intake;
    private Jukebox _jukebox;
    private VisionSystem _visionSystem;

    private PathFollower _pathFollower;

    private double _targetAbsoluteAngle;

    private boolean _pathInitialized;

    EasyAuton(String autoName, AutoCmds[] auton) {
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        _visionSystem = VisionSystem.getInstance();

        _pathFollower = new PathFollower(autoName);
    }

    public static AutoCmds[] getInternalAuton(String internalKey) {
        int internalIndex = -1;
        for (int i = 0; i < _internalKeys.length; i++) {
            if (internalKey == _internalKeys[i]) {
                internalIndex = i;

                break;
            }
        }

        if (internalIndex < 0) return new AutoCmds[] {};

        int[] auton = _internalAutons[internalIndex];

        return auton;
    }

    public void execute() {
        AutoCmds stepFunction = _auton[_step];

        switch (stepFunction) {
            case FOLLOW:
                follow();
                break;

            case FOLLOW_WITH_POSE_AIM:
                followWithPoseAim();
                break;
        }
    }

    private void follow() {
        follow(false);
    }

    private void followWithPoseAim() {
        follow(true);
    }

    private void follow(boolean poseAim) {
        if (!_pathInitialized) {
            _pathFollower.startNextPath(new ChassisSpeeds(),
                                        _drivetrain.getGyroRotation());

            _pathInitialized = true;

            return;
        }

        ChassisSpeeds chassisSpeeds = _pathFollower.getPathTarget(
            _drivetrain.getPose()
        );

        if (poseAim) {
            // double[] targetInfo = _visionSystem.getTargetingInfo();

            // double validTargets = targetInfo[0];
            // double targetAngle = targetInfo[2];
    
            // if (validTargets > 0.0) {
            //     _targetAbsoluteAngle = _drivetrain.getAbsoluteTargetAngle(
            //         targetAngle
            //     );
            // }
    
            // rotation = _drivetrain.getAngleToTarget(_targetAbsoluteAngle);

            // chassisSpeeds = new ChassisSpeeds(
            //     chassisSpeeds.vxMetersPerSecond,
            //     chassisSpeeds.vyMetersPerSecond,
            //     chassisSpeeds.omegaRadiansPerSecond
            // );
        }

        // this gives us chassis speeds, we need to modify that rotation though
        _drivetrain.drive(chassisSpeeds);

        if (_pathFollower.isPathFinished()) {
            _drivetrain.drive(new ChassisSpeeds());

            _pathInitialized = false;

            _step++;
        }
    }

    public void getPoseAimRotation(ChassisSpeeds chassisSpeeds) {
        double[] targetInfo = _visionSystem.getTargetingInfo();

        double validTargets = targetInfo[0];
        double targetAngle = targetInfo[2];

        if (validTargets > 0.0) {
            _targetAbsoluteAngle = _drivetrain.getAbsoluteTargetAngle(
                targetAngle
            );
        }

        rotation = _drivetrain.getAngleToTarget(_targetAbsoluteAngle);

        chassisSpeeds = new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond
        );
    }
}
