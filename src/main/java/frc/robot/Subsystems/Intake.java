package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Enums.IntakeState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private static Intake _instance;

    private CANSparkMax _motor;

    private IntakeState _currentState = IntakeState.STOP;

    //WILL BE CHANGED TO REFLECT ACTUAL VALUES
    private double _intakeSpeed = 1;
    private double _ejectSpeed = -1;
    private double _stopped = 0;
    private double _passiveEjectSpeed = -0.5;

    Intake() {
        _motor = new CANSparkMax(Constants.kIntakeMotorId,
                                 MotorType.kBrushless);
    }
    

    public static Intake getInstance() {
        if (_instance == null) {
            _instance = new Intake();
        }

        return _instance;
    }

    public void stop() {
        _motor.set(_stopped);
    }
    
    public void intake() {
        _motor.set(_intakeSpeed);
    }

    public void eject() {
        _motor.set(_ejectSpeed);
    }

    public void passiveEject() {
        _motor.set(_passiveEjectSpeed);
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case STOP:
                stop();

                break;
            
            case INTAKE:
                intake();

                // pseudocode if we have to interpret the jukebox data vs the
                // jukebox sending us states 

                // if (Jukebox.getInstance().hasNote()) {
                //     setWantedState(IntakeState.PASSIVE_EJECT);
                // }

                break;

            case PASSIVE_EJECT:
                passiveEject();

                break;

            case EJECT:
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

    //@Override
    public void logTelemetry()
    {
        SmartDashboard.putString("intakeMotorState", _currentState.name());

        SmartDashboard.putNumber("intakeMotorTemperature",
                                 _motor.getMotorTemperature());

        SmartDashboard.putNumber("intakeMotorOutputCurrent",
                                 _motor.getOutputCurrent());
        
    }

    


}

