package frc.robot.Utilities;

import com.ctre.phoenix.led.CANdle;
import frc.robot.Constants.Constants;

public class LEDController {
    private static LEDController _instance;
    private CANdle candle;

    public LEDController()
    {
        candle = new CANdle(Constants.kCANdleId);
    }

    public static LEDController getInstance()
    {
        if (_instance != null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }
}
