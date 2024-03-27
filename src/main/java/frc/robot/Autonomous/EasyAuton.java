package frc.robot.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Constants;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Jukebox;
import frc.robot.Subsystems.VisionSystem;
import frc.robot.Utilities.PathFollower;

public class EasyAuton extends AutonomousMode {
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
    private static final AutoCmds[][] _internalAutons = {};

    private AutoCmds[] _auton;

    private int _step = 0;

    private Drivetrain _drivetrain;
    private Intake _intake;
    private Jukebox _jukebox;
    private VisionSystem _visionSystem;

    private PathFollower _pathFollower;

    private double _targetAbsoluteAngle;

    private boolean _pathInitialized;

    private boolean _pathFinished;

    EasyAuton(String autoName, AutoCmds[] auton) {
        _drivetrain = Drivetrain.getInstance();
        _intake = Intake.getInstance();
        _jukebox = Jukebox.getInstance();
        _visionSystem = VisionSystem.getInstance();

        _pathFollower = new PathFollower(autoName);

        _auton = auton;
    }

    EasyAuton(String autoName, String internalKey) {
        this(autoName, getInternalAuton(internalKey));
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

        AutoCmds[] auton = _internalAutons[internalIndex];

        return auton;
    }

    @Override
    public boolean isComplete() {
        return _step + 1 > _auton.length;
    }

    @Override
    public void execute() {
        AutoCmds stepFunction = _auton[_step];

        switch (stepFunction) {
            case FOLLOW_WITH_POSE_AIM:
                if (followWithPoseAim()) {
                    reset();

                    break;
                };

            case FOLLOW_WITH_SHOOT_SPKLL_AIM:
                if (follow() && shootSpkLLAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_SHOOT_SPKPOD_AIM:
                if (follow() && shootSpkPodAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_SHOOT_SPKLIN_AIM:
                if (follow() && shootSpkLinAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_SHOOT_SPKSUB_AIM:
                if (follow() && shootSpkSubAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_FULL_SPKLL_AIM:
                if (followWithPoseAim() && shootSpkLLAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_FULL_SPKPOD_AIM:
                if (followWithPoseAim() && shootSpkPodAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_FULL_SPKLIN_AIM:
                if (followWithPoseAim() && shootSpkLinAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_FULL_SPKSUB_AIM:
                if (followWithPoseAim() && shootSpkSubAim()) {
                    reset();

                    break;
                }

            case FOLLOW_WITH_PASSTHROUGH:
                // _jukebox.setState(JukeboxEnum.PASSTHROUGH);

                if (follow()) {
                    reset();

                    break;
                }

            case FOLLOW:
                _jukebox.setState(JukeboxEnum.IDLE);

                if (follow()) {
                    reset();

                    break;
                }

            case SCORE:
                if (score()) {
                    reset();

                    break;
                }

            case POSE_AIM:
                if (poseAim()) {
                    reset();

                    break;
                }

            case FULL_SPKLL_AIM:
                if (poseAim() && shootSpkLLAim()) {
                    reset();

                    break;
                }

            case FULL_SPKPOD_AIM:
                if (poseAim() && shootSpkPodAim()) {
                    reset();

                    break;
                }

            case FULL_SPKLIN_AIM:
                if (poseAim() && shootSpkLinAim()) {
                    reset();

                    break;
                }

            case FULL_SPKSUB_AIM:
                if (poseAim() && shootSpkSubAim()) {
                    reset();

                    break;
                }

            case SHOOT_SPKLL_AIM:
                if (shootSpkLLAim()) {
                    reset();

                    break;
                }

            case SHOOT_SPKPOD_AIM:
                if (shootSpkPodAim()) {
                    reset();

                    break;
                }

            case SHOOT_SPKLIN_AIM:
                if (shootSpkLinAim()) {
                    reset();

                    break;
                }

            case SHOOT_SPKSUB_AIM:
                if (shootSpkSubAim()) {
                    reset();

                    break;
                }
        }
    }

    private void reset() {
        _pathInitialized = false;

        _step++;
    }

    private boolean follow() {
        return follow(false);
    }

    private boolean followWithPoseAim() {
        return follow(true);
    }

    private boolean follow(boolean poseAim) {
        if (!_pathInitialized) {
            _pathFollower.startNextPath(new ChassisSpeeds(),
                                        _drivetrain.getPose());

            _pathInitialized = true;

            return false;
        }

        ChassisSpeeds chassisSpeeds = _pathFollower.getPathTarget(
            _drivetrain.getPose()
        );

        if (poseAim) {
            double rotation = -(_visionSystem.getTargetAngle() / 105);

            // TODO: double check rotation calc is right
            chassisSpeeds = new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                rotation * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND
            );

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

            return true;
        }

        return false;
    }

    private boolean poseAim() {
        double rotation = -(_visionSystem.getTargetAngle() / 105);

        if (rotation == 0) return true;

        // TODO: double check rotation calc is right
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            0,
            0,
            rotation * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND
        );
        
        _drivetrain.drive(chassisSpeeds);

        return false;
    }

    private boolean shootAim(JukeboxEnum state) {
        _jukebox.setState(state);

        return _jukebox.getSpeakerShotReady();
    }

    private boolean shootSpkLLAim() {
        return shootAim(JukeboxEnum.PREP_SPEAKER);
    }

    private boolean shootSpkLinAim() {
        return shootAim(JukeboxEnum.PREP_SPEAKER_LINE);
    }

    private boolean shootSpkSubAim() {
        return shootAim(JukeboxEnum.PREP_SPEAKER_SUBWOOFER);
    }

    private boolean shootSpkPodAim() {
        return shootAim(JukeboxEnum.PREP_SPEAKER_PODIUM);
    }

    private boolean score() {
        _jukebox.setState(JukeboxEnum.SCORE);

        return !_jukebox.hasNote();
    }
}
