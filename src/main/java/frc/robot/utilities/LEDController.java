package frc.robot.Utilities;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import frc.robot.Constants.Constants;

public class LEDController {

    private static LEDController _instance;
    private CANdle underCandle;
    private CANdle jukeboxCandle1;
    private CANdleConfiguration config;
    private SingleFadeAnimation idleAnimation;
    private Animation packingHeat;
    private Animation fire;
    private Animation climb;
    private int underNumLeds;
    private int jukeboxNumLeds;

    public LEDController()
    {
        underNumLeds = 100;
        jukeboxNumLeds = 100;
        underCandle = new CANdle(0);
        jukeboxCandle1 = new CANdle(1);
        config = new CANdleConfiguration();
        // not holding note
        idleAnimation = new SingleFadeAnimation(0, 0, 255, 0, .5, underNumLeds);
        // holding note
        packingHeat = new FireAnimation(.5, .5, underNumLeds, 1, .5, false, 0);
        // shooter animation
        fire = new ColorFlowAnimation(0, 255, 0, 0, .1, jukeboxNumLeds, Direction.Forward);
        // climbing animation
        climb = new ColorFlowAnimation(8, 255, 255, 0, .1, jukeboxNumLeds, Direction.Forward);
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
    
    public void setLEDs(int red, int green, int blue)
    {
        underCandle.setLEDs(red, green, blue);
    }
    /**
     * Turns the brightness of the candle to 0 basically off.
     */
    public void off()
    {
        config.brightnessScalar = 0;
    }

    public void idleLights()
    {
        underCandle.animate(idleAnimation);
        jukeboxCandle1.animate(idleAnimation);
    }

    public void holdingPieceLights()
    {
        jukeboxCandle1.animate(packingHeat);
    }

    
    public void climbingLights()
    {}



    public void fireAwayLights()
    {
        jukeboxCandle1.animate(fire);
    }
}
