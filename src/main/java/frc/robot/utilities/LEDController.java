package frc.robot.Utilities;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import frc.robot.Constants.Constants;

public class LEDController {

    private static LEDController _instance;
    private CANdle idleCandle;
    private CANdle stateCandle1;
    private CANdle stateCandle2;
    private CANdleConfiguration config;
    private Animation idle;
    private Animation packingHeat;
    private Animation fire;
    private int numLeds;

    public LEDController()
    {
        numLeds = 100;
        stateCandle1 = new CANdle(Constants.kCANdleId);
        config = new CANdleConfiguration();
        idle = new SingleFadeAnimation(0, 0, 255);
        packingHeat = new FireAnimation(.5, .5, numLeds, 1, .5, false, 0);
        fire = new ColorFlowAnimation(0, 255, 0, 0, .1, numLeds, Direction.Forward);
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
        idleCandle.setLEDs(red, green, blue);
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
        idleCandle.animate(idle);
        stateCandle1.animate(idle);
        stateCandle2.animate(idle);
    }

    public void holdingPieceLights()
    {
        stateCandle1.animate(packingHeat);
        stateCandle2.animate(packingHeat);
    }

    public void fireAwayLights()
    {
        stateCandle1.animate(fire);
        stateCandle2.animate(fire);
    }
}
