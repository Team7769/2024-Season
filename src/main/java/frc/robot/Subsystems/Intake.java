package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Enums.IntakeState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{
    private static Intake _instance;

    private CANSparkMax _motor;

    private IntakeState _currentState = IntakeState.STOP;

    //WILL BE CHANGED TO REFLECT ACTUAL VALUES
    private final double kIntakeSpeed = .5;
    private final double kEjectSpeed = -.5;
    private final double kStopSpeed = 0;
    private final double kPassiveEjectSpeed = -0.25;

    private final int kMotorStallLimit = 80;
    private final int kMotorFreeLimit = 100;
    private final boolean kInverted = false;

    private Jukebox _jukebox;

    Intake() {
        _motor = new CANSparkMax(Constants.kIntakeMotorId,
                                 MotorType.kBrushless);

        _motor.setIdleMode(IdleMode.kBrake);
        _motor.setSmartCurrentLimit(kMotorStallLimit, kMotorFreeLimit);
        _motor.setInverted(kInverted);
        _motor.burnFlash();

        _jukebox = Jukebox.getInstance();
    }
    

    public static Intake getInstance() {
        if (_instance == null) {
            _instance = new Intake();
        }

        return _instance;
    }

    public void stop() {
        _motor.set(kStopSpeed);
    }
    
    public void intake() {
        // if we have a note, change to passive eject mode
        if (_jukebox.hasNote()) {
            setWantedState(IntakeState.PASSIVE_EJECT);

            return;
        }

        _motor.set(kIntakeSpeed);
    }

    // emergency eject
    public void eject() {
        _motor.set(kEjectSpeed);
    }

    // when we have a note, slowly turn the motor in reverse to avoid sucking
    // notes in
    public void passiveEject() {
        // if we dont have a note, change to intake mode
        if (!_jukebox.hasNote()) {
            setWantedState(IntakeState.INTAKE);

            return;
        }

        _motor.set(kPassiveEjectSpeed);
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case STOP:
                stop();

                break;
            
            case INTAKE:
                intake();

                break;

            case PASSIVE_EJECT:
                // when we have a note, slowly turn the motor in reverse to 
                // avoid sucking notes in
                passiveEject();

                break;

            case EJECT:
                // emergency eject
                eject();

                break;

            default:
                break;
        }
    }

    public void setWantedState(IntakeState state) {
        if (state == _currentState) return;

        // switch (state) {
        //     case STOP:
        //         stop();

        //         break;
            
        //     case INTAKE:
        //         intake();

        //         break;

        //     case PASSIVE_EJECT:
        //         passiveEject();

        //         break;

        //     case EJECT:
        //         eject();

        //         break;

        //     default:
        //         stop(); // better safe than sorry

        //         break;
        // }

        // may want to move this before function calls
        _currentState = state;
    }

    public IntakeState getState() {
        return _currentState;
    }

    @Override
    public void logTelemetry()
    {
        SmartDashboard.putString("intakeMotorState", _currentState.name());
        SmartDashboard.putNumber("intakeMotorTemperature",
                                 _motor.getMotorTemperature());
        SmartDashboard.putNumber("intakeMotorOutputCurrent",
                                 _motor.getOutputCurrent());
        SmartDashboard.putNumber("intakeMotorOutput", _motor.get());
        SmartDashboard.putNumber("intakeMotorPosition", _motor.getEncoder().getPosition());
        SmartDashboard.putNumber("intakeMotorVelocity", _motor.getEncoder().getVelocity());
    }
}

