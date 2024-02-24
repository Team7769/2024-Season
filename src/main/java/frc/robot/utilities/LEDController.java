package frc.robot.Utilities;

import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Enums.JukeboxEnum;

public class LEDController {

    private static LEDController _instance;

    private CANdle upperCandle;
    private CANdle lowerCandle;

    private CANdleConfiguration config;
    private SingleFadeAnimation idleAnimation;

    private Optional<Alliance> _alliance;

    private Animation packingHeat;
    private Animation fire;
    private Animation climbLights;

    private int underNumLeds;
    private int jukeboxNumLeds;

    public LEDController()
    {
        underNumLeds = 100;
        jukeboxNumLeds = 100;
        upperCandle = new CANdle(15);
        lowerCandle = new CANdle(0); // TBD
        config = new CANdleConfiguration();

        // holding note
        packingHeat = new FireAnimation(.5, .5, underNumLeds, 1, .5, false, 0);
        // shooter animation
        fire = new ColorFlowAnimation(0, 255, 0, 0, .1, jukeboxNumLeds, Direction.Forward);
        // climbing animation
        climbLights = new ColorFlowAnimation(8, 255, 255, 0, .1, jukeboxNumLeds, Direction.Forward);
    }


    public static LEDController getInstance()
    {
        if (_instance != null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }
    /**
     * sets the brightness of the strip
     * @param brightness a number from 0 to 1 enter a precentage
     */
    public void setBrightness(double brightness)
    {
        config.brightnessScalar = brightness;
    }

    /**
     * Method that sets the color of the leds with a value from 0 - 255
     * @param red red value
     * @param green green value
     * @param blue blue value
     */
    
    public void setUpperLEDs(int red, int green, int blue)
    {
        upperCandle.setLEDs(red, green, blue);
    }
    public void setLowerLEDs(int red, int green, int blue)
    {
        lowerCandle.setLEDs(red, green, blue);
    }
    /**
     * Turns the brightness of the candle to 0 basically off.
     */
    public void off()
    {
        config.brightnessScalar = 0;
    }

    public void holdingPieceLights()
    {
        upperCandle.animate(packingHeat);
    }

    
    public void climbingLights()
    {
        upperCandle.animate(climbLights);
    }

    public void climbIsFinishedLights()
    {
        upperCandle.animate(null);
    }



    public void fireAwayLights()
    {
        upperCandle.animate(fire);
    }

    public void handleLights(JukeboxEnum jukeboxCurrentState)
    {
        switch (jukeboxCurrentState) {
            case IDLE:
                break;
            case SCORE:
                break;
            case PREP_SPEAKER:
                break;
            case PREP_AMP:
                break;
            case PREP_TRAP:
                break;
            case RESET:
                break;
            case EXTEND_FOR_CLIMB:
                break;
            case CLIMB:
                break;
            case MANUAL:
                break;
            default:
                _alliance = DriverStation.getAlliance();
                if (_alliance.get() == DriverStation.Alliance.Blue) {
                    upperCandle.setLEDs(0, 0, 255);   
                } else {
                    upperCandle.setLEDs(255, 0, 0);
                }
                break;
        }
    }
}
