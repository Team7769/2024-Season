package frc.robot.Utilities;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import frc.robot.Subsystems.Jukebox;

public class LEDController {

    private static LEDController _instance;

    private CANdle upperCandle;
    // private CANdle lowerCandle;

    private CANdleConfiguration config;

    // private Optional<Alliance> _alliance;

    // Animation for Jukebox state
    private Animation PREP_SPEAKER_LIGHTS;
    private Animation PREP_AMP_LIGHTS;
    private Animation PREP_TRAP_LIGHTS;
    private Animation EXTEND_FOR_CLIMB_LIGHTS;    
    private Animation CLIMB_LIGHTS;
    private Animation IDLE_LIGHTS;

    // private int underNumLeds;
    private int jukeboxNumLeds;

    private Jukebox jukebox;

    /**
     * The are grb lights
     */

    LEDController()
    {
        jukebox = Jukebox.getInstance();
        upperCandle = new CANdle(15);
        // lowerCandle = new CANdle(0); // TBD
        // underNumLeds = 400;
        upperCandle.setLEDs(0, 255, 0, 0, 0, jukeboxNumLeds);
        jukeboxNumLeds = 50;    
        IDLE_LIGHTS = new StrobeAnimation(0, 255, 0, 0, .15, jukeboxNumLeds);
        PREP_SPEAKER_LIGHTS = new FireAnimation(.5, .5, jukebox.getShooterLeds(jukeboxNumLeds), .25, .1);
        PREP_AMP_LIGHTS = new RainbowAnimation(.5, .5, jukeboxNumLeds);
        PREP_TRAP_LIGHTS = new ColorFlowAnimation(255, 165, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        EXTEND_FOR_CLIMB_LIGHTS = new ColorFlowAnimation(230, 230, 250, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        CLIMB_LIGHTS = new ColorFlowAnimation(255, 215, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
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
    // public void handleBottomLights()
    // {
    //     _alliance = DriverStation.getAlliance();
    //     if (_alliance.get() == DriverStation.Alliance.Blue) {
    //         lowerCandle.setLEDs(0, 0, 255);   
    //     } else {
    //         lowerCandle.setLEDs(255, 0, 0);
    //     }
    // }

    /**
     * For each state in JukeBox a light will come on
     */
    public void handleLights()
    {
        var currentState = jukebox.getState();
        switch (currentState) {
            case IDLE:
                if (jukebox.hasNote())
                {
                    upperCandle.clearAnimation(0);
                    upperCandle.setLEDs(0, 255, 0, 0, 0, jukeboxNumLeds);
                } else {
                    upperCandle.animate(IDLE_LIGHTS,0);
                }
                break;
            case SCORE:
                upperCandle.setLEDs(0, 255, 255, 0, 0, jukeboxNumLeds);
                break;
            case PREP_LAUNCH:
            case PREP_SPEAKER_PODIUM:
            case PREP_SPEAKER_LINE:
            case PREP_SPEAKER:
                upperCandle.clearAnimation(0);
                upperCandle.animate(PREP_SPEAKER_LIGHTS, 0);
                break;
            case PREP_AMP:
                upperCandle.clearAnimation(0);
                upperCandle.animate(PREP_AMP_LIGHTS,0);
                break;
            case PREP_TRAP:
                upperCandle.clearAnimation(0);
                upperCandle.animate(PREP_TRAP_LIGHTS, 0);
                break;
            case RESET:
            case EXTEND_FOR_CLIMB:
                upperCandle.clearAnimation(0);
                upperCandle.animate(EXTEND_FOR_CLIMB_LIGHTS, 0);
                break;
            case CLIMB:
                upperCandle.clearAnimation(0);
                upperCandle.animate(CLIMB_LIGHTS, 0);
                break;
            case MANUAL:
            default:
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            }


        
    }
}
