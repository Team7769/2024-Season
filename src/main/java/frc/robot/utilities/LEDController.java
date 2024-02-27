package frc.robot.Utilities;

import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Jukebox;

public class LEDController {

    private static LEDController _instance;

    private CANdle upperCandle;
    private CANdle lowerCandle;

    private CANdleConfiguration config;

    private Optional<Alliance> _alliance;

    // Animation for Jukebox state
    private Animation IDLE_LIGHTS;
    private Animation SCORE_LIGHTS;
    private Animation PREP_SPEAKER_LIGHTS;
    private Animation PREP_AMP_LIGHTS;
    private Animation PREP_TRAP_LIGHTS;
    private Animation RESET_LIGHTS;
    private Animation EXTEND_FOR_CLIMB_LIGHTS;    
    private Animation CLIMB_LIGHTS;
    private Animation MANUAL_LIGHTS;



    private int underNumLeds;
    private int jukeboxNumLeds;

    private Jukebox jukebox;
    private JukeboxEnum _lastState;

    LEDController()
    {
        underNumLeds = 40;
        jukeboxNumLeds = 40;
        upperCandle = new CANdle(15);
        lowerCandle = new CANdle(25); // TBD
        config = new CANdleConfiguration();
        
        setBrightness(.1);
        config.stripType = LEDStripType.RGB;
        config.vBatOutputMode = VBatOutputMode.On;
        config.brightnessScalar = 0.5;
        config.disableWhenLOS = false;
        config.statusLedOffWhenActive = false;
        upperCandle.configAllSettings(config, 100);

        // lowerCandle.configAllSettings(config);
        
        // animation for IDLE --color is Pure White
        IDLE_LIGHTS = new ColorFlowAnimation(0, 0, 0, 255, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for SCORE --color is Green
        SCORE_LIGHTS = new ColorFlowAnimation(0, 255, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for PREP_SPEAKER --color is Turquoise
        PREP_SPEAKER_LIGHTS = new ColorFlowAnimation(0, 255, 255, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for PREP_AMP --color is Indigo
        PREP_AMP_LIGHTS = new ColorFlowAnimation(75, 0, 130, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for PREP_TRAP --color is Sunset Orange
        PREP_TRAP_LIGHTS = new ColorFlowAnimation(255, 165, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for RESET --color is Magenta
        RESET_LIGHTS = new ColorFlowAnimation(255, 0, 255, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for EXTEND_FOR_CLIMB --color is Lavender
        EXTEND_FOR_CLIMB_LIGHTS = new ColorFlowAnimation(230, 230, 250, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for CLIMB --color is Gold
        CLIMB_LIGHTS = new ColorFlowAnimation(255, 215, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for MANUAL --color is Purple
        MANUAL_LIGHTS = new ColorFlowAnimation(128, 0, 128, 0, 0.5, jukeboxNumLeds, Direction.Forward);

        jukebox = Jukebox.getInstance();
    }


    public static LEDController getInstance()
    {
        if (_instance == null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }
    /**
     * sets the brightness of the strip
     * @param brightness a number from 0 to 1 enter a precentage
     */
    private void setBrightness(double brightness)
    {
        config.brightnessScalar = brightness;
    }

    
    /**
     * Turns the brightness of the candle to 0 basically off.
     */
    public void off()
    {
        config.brightnessScalar = 0;
    }

    /**
     * Handles the bottom lights.
     * If _alliance is blue then make the lights blue
     * If _alliance is red then make the lights red
     */
    public void handleBottomLights()
    {
        _alliance = DriverStation.getAlliance();
        if (_alliance.get() == DriverStation.Alliance.Blue) {
            lowerCandle.setLEDs(0, 0, 255);   
        } else {
            lowerCandle.setLEDs(255, 0, 0);
        }
    }

    /**
     * For each state in JukeBox a light will come on
     */
    public void handleLights()
    {
        var currentState = jukebox.getState();
        if (_lastState != currentState) {
            switch (currentState) {
            case IDLE:
                //upperCandle.setLEDs(100, 100, 100, 100, 0, 25);
                upperCandle.animate(IDLE_LIGHTS);
                break;
            case SCORE:
                upperCandle.animate(SCORE_LIGHTS);
                break;
            case PREP_SPEAKER:
                upperCandle.animate(PREP_SPEAKER_LIGHTS);
                break;
            case PREP_AMP:
                upperCandle.animate(PREP_AMP_LIGHTS);
                break;
            case PREP_TRAP:
                upperCandle.animate(PREP_TRAP_LIGHTS);
                break;
            case RESET:
                upperCandle.animate(RESET_LIGHTS);
                break;
            case EXTEND_FOR_CLIMB:
                upperCandle.animate(EXTEND_FOR_CLIMB_LIGHTS);
                break;
            case CLIMB:
                upperCandle.animate(CLIMB_LIGHTS);
                break;
            case MANUAL:
                upperCandle.animate(MANUAL_LIGHTS);
                break;
            default:
                upperCandle.animate(IDLE_LIGHTS);
                break;
        }
        _lastState = currentState;
        }
        
    }
}
