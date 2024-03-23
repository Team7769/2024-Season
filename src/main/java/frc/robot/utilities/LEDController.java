package frc.robot.Utilities;

import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Constants;
import frc.robot.Enums.JukeboxEnum;
import frc.robot.Subsystems.Jukebox;

public class LEDController {

    private static LEDController _instance;

    private CANdle upperCandle;
    private CANdle lowerCandle;

    private CANdleConfiguration config;

    private Optional<Alliance> _alliance;

    // Animation for Jukebox state
    private Animation SCORE_LIGHTS;
    private Animation PREP_SPEAKER_LIGHTS;
    private Animation PREP_AMP_LIGHTS;
    private Animation PREP_TRAP_LIGHTS;
    private Animation RESET_LIGHTS;
    private Animation EXTEND_FOR_CLIMB_LIGHTS;    
    private Animation CLIMB_LIGHTS;
    private Animation MANUAL_LIGHTS;
    private Animation IDLE_LIGHTS;

    private int underNumLeds;
    private int jukeboxNumLeds;

<<<<<<< HEAD
    private boolean hasAnimation;

    private Jukebox jukebox;
=======
    private Jukebox _jukebox;
>>>>>>> 120bf681f41c0ad1418646b1cc2d0070a191f300

    /**
     * The are grb lights
     */

    LEDController()
    {
<<<<<<< HEAD

        hasAnimation = false;

        jukebox = Jukebox.getInstance();
        upperCandle = new CANdle(15);
        // lowerCandle = new CANdle(0); // TBD
=======
        _jukebox = Jukebox.getInstance();
        upperCandle = new CANdle(Constants.kUpperCandle);
        lowerCandle = new CANdle(Constants.kLowerCandle);
>>>>>>> 120bf681f41c0ad1418646b1cc2d0070a191f300
        // underNumLeds = 400;
        upperCandle.setLEDs(0, 255, 0, 0, 0, jukeboxNumLeds);
        jukeboxNumLeds = 50;        
        // animation for IDLE --color is Pure White
        IDLE_LIGHTS = new StrobeAnimation(255, 255, 0, 0, .15, jukeboxNumLeds);
        // animation for SCORE --color is Green
        SCORE_LIGHTS = new RainbowAnimation(1, 5, jukeboxNumLeds, true, 0);
        // animation for PREP_SPEAKER --color is Turquoise
<<<<<<< HEAD
        PREP_SPEAKER_LIGHTS = new FireAnimation(.5, .5, jukebox.getShooterLeds(jukeboxNumLeds), .25, .1);
=======
        PREP_SPEAKER_LIGHTS = new FireAnimation(.5, .5, _jukebox.getShooterLeds(jukeboxNumLeds), .25, .1);
        // animation for PREP_AMP --color is Indigo
        PREP_AMP_LIGHTS = new RainbowAnimation(.5, .5, jukeboxNumLeds);
>>>>>>> 120bf681f41c0ad1418646b1cc2d0070a191f300
        // animation for PREP_TRAP --color is Sunset Orange
        PREP_TRAP_LIGHTS = new ColorFlowAnimation(255, 165, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for RESET --color is Magenta
        RESET_LIGHTS = new ColorFlowAnimation(255, 0, 255, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for EXTEND_FOR_CLIMB --color is Lavender
        EXTEND_FOR_CLIMB_LIGHTS = new ColorFlowAnimation(230, 230, 250, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for CLIMB --color is Gold
        CLIMB_LIGHTS = new FireAnimation(.5,.5,jukeboxNumLeds,.25,.1);
        // animation for MANUAL --color is Purple
        MANUAL_LIGHTS = new ColorFlowAnimation(128, 0, 128, 0, 0.5, jukeboxNumLeds, Direction.Forward);



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
     * For each state in JukeBox a light will come on
     */
    public void handleLights()
    {
        _alliance = DriverStation.getAlliance();
        if (_alliance.isPresent() && _alliance.get() == DriverStation.Alliance.Blue) {
            lowerCandle.setLEDs(0, 0, 255, 0, 0, jukeboxNumLeds); 
        } else {
            lowerCandle.setLEDs(255, 0, 0, 0, 0, jukeboxNumLeds); 
        }

        var currentState = _jukebox.getState();
        switch (currentState) {
            case IDLE:
                if (_jukebox.hasNote())
                {
                    upperCandle.setLEDs(0, 255, 0, 0, 0, jukeboxNumLeds);
                } else {
                    upperCandle.animate(IDLE_LIGHTS);
                    hasAnimation = true;
                }
                break;
            case SCORE:
                upperCandle.animate(SCORE_LIGHTS);
                hasAnimation = true;
                break;
            case PREP_LAUNCH:
            case PREP_SPEAKER_PODIUM:
            case PREP_SPEAKER_LINE:
            case PREP_SPEAKER:
<<<<<<< HEAD
                upperCandle.setLEDs(0, 0, 128, 0, 0, jukebox.getShooterLeds(jukeboxNumLeds));
=======
                // upperCandle.clearAnimation(0);
                // upperCandle.animate(PREP_SPEAKER_LIGHTS);
                upperCandle.setLEDs(255, 0, 0, 0, 0, _jukebox.getShooterLeds(jukeboxNumLeds));
>>>>>>> 120bf681f41c0ad1418646b1cc2d0070a191f300
                break;
            case PREP_AMP:
                upperCandle.setLEDs(128, 0, 128, 0, 0, jukeboxNumLeds);
                break;
            case PREP_TRAP:
                upperCandle.clearAnimation(0);
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            case RESET:
                upperCandle.setLEDs(45, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            case EXTEND_FOR_CLIMB:
<<<<<<< HEAD
                upperCandle.animate(EXTEND_FOR_CLIMB_LIGHTS);
                hasAnimation = true;
                break;
            case CLIMB:
                upperCandle.animate(CLIMB_LIGHTS);
                hasAnimation = true;
=======
                upperCandle.clearAnimation(0);
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            case CLIMB:
                if (_jukebox.getElevatorPosition() < .05) {
                    upperCandle.animate(CLIMB_LIGHTS);
                }
>>>>>>> 120bf681f41c0ad1418646b1cc2d0070a191f300
                break;
            case MANUAL:
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            default:
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            }

        if (hasAnimation)
        {
            upperCandle.clearAnimation(0);
            hasAnimation=false;
        }
    }

}
