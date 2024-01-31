package frc.robot.Utilities;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants.Constants;

public class LEDController {

    private static LEDController _instance;
    private CANdle candle;
    private CANdleConfiguration config;

    public LEDController()
    {
        candle = new CANdle(Constants.kCANdleId);
        config = new CANdleConfiguration();
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

    public void setLEDs(int red, int green, int blue)
    {
        candle.setLEDs(red, green, blue);
    }

    public void off()
    {
        config.brightnessScalar = 0;
    }
}
