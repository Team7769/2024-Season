package frc.robot.Autonomous;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private static Translation2d kSpeaker;

    private static final double kMaxRotError = 0.25;

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
    public void initialize() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            kSpeaker = alliance.get() == Alliance.Blue ?
                Constants.kBlueSpeaker :
                Constants.kRedSpeaker;
        }
    }

    @Override
    public boolean isComplete() {
        return _step + 1 > _auton.length;
    }

    @Override
    public void execute() {
        AutoCmds stepFunction = _auton[_step];

        if (_jukebox.getState() == JukeboxEnum.SCORE && !_jukebox.hasNote()) {
            _jukebox.setState(JukeboxEnum.IDLE);
        }

        boolean pathFinished;

        switch (stepFunction) {
            case FOLLOW:
                if (_jukebox.getState() != JukeboxEnum.SCORE) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                }

                if (follow()) {
                    reset();
                }

                break;

            case FOLLOW_WITH_ROT_AIM:
                if (_jukebox.getState() != JukeboxEnum.SCORE) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                }

                if (followWithRotAim()) {
                    reset();
                }

                break;

            case FOLLOW_WITH_JBX_AIM:
                pathFinished = follow();

                if (pathFinished) {
                    if (_jukebox.hasNote()) {
                        if (jbxAim()) {
                            reset();
                        }
                    } else {
                        reset();
                    }
                };

                break;

            case FOLLOW_WITH_FULL_AIM:  
                pathFinished = followWithRotAim();

                if (pathFinished) {
                    if (_jukebox.hasNote()) {
                        if (jbxAim()) {
                            reset();
                        }
                    } else {
                        reset();
                    }
                };

                break;

            case FOLLOW_WITH_PASSTHROUGH:
                break;

            case FULL_AIM:
                if (rotAim()) {
                    if (_jukebox.hasNote()) {
                        if (jbxAim()) {
                            reset();
                        }
                    } else {
                        reset();
                    }
                }

                break;

            case ROT_AIM:
                if (_jukebox.getState() != JukeboxEnum.SCORE) {
                    _jukebox.setState(JukeboxEnum.IDLE);
                }

                if (rotAim()) {
                    reset();
                }

                break;

            case JBX_AIM:
                if (_jukebox.hasNote()) {
                    if (jbxAim()) {
                        reset();
                    }
                } else {
                    reset();
                }

                break;

            case SHOOT:
                _jukebox.setState(JukeboxEnum.SCORE);

                if (!_jukebox.hasNote()) {
                    reset();
                }

                break;

            case SHOOT_RUSHED:
                _jukebox.setState(JukeboxEnum.SCORE);

                reset();

                break;

        }
    }

    private void reset() {
        _pathInitialized = false;

        _step++;
    }

    private boolean follow() {
        return follow(false);
    }

    private boolean followWithRotAim() {
        return follow(true);
    }

    private boolean follow(boolean rotAim) {
        if (!_pathInitialized) {
            _pathFollower.startNextPath(new ChassisSpeeds(),
                                        _drivetrain.getPose());

            _pathInitialized = true;

            return false;
        }

        ChassisSpeeds chassisSpeeds = _pathFollower.getPathTarget(
            _drivetrain.getPose()
        );

        if (rotAim) {
            double angle = kSpeaker
                .minus(_drivetrain.getPose().getTranslation())
                .getAngle()
                .minus(_drivetrain.getGyroRotation())
                .getDegrees();

            double rotation = angle / 105;

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

    private boolean rotAim() {
        double rotation = -(_visionSystem.getTargetAngle() / 105);

        if (rotation < kMaxRotError) return true;

        // TODO: double check rotation calc is right
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            0,
            0,
            rotation * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND
        );
        
        _drivetrain.drive(chassisSpeeds);

        return false;
    }

    private boolean jbxAim() {
        _jukebox.setState(JukeboxEnum.PREP_SPEAKER);

        return _jukebox.isPivotReady() && _jukebox.isShooterReady();
    }

    private boolean score() {
        _jukebox.setState(JukeboxEnum.SCORE);

        return !_jukebox.hasNote();
    }
}
