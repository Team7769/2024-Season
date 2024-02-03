package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Enums.IntakeState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    private static Intake _instance;

    private CANSparkMax _motor;

    private IntakeState _currentState = IntakeState.DISABLED;

    private double _intakeSpeed = 0;
    private double _reverseSpeed = 0;

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

    public void disable() {
        _motor.set(0);
    }
    
    public void intake() {
        _motor.set(_intakeSpeed);
    }

    public void reverse() {
        _motor.set(_reverseSpeed);
    }

    // public void setState(IntakeState state) {
    //     switch (state) {
    //         case IntakeState.Disabled
    //     }
    // }

    @Override
    public void logTelemetry()
    {
        SmartDashboard.putString("intakeState", _currentState.name());
        SmartDashboard.putNumber("intakeTemperature", _motor.getMotorTemperature());
        
        
    }

    


}

