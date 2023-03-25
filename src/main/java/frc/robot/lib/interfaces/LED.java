package frc.robot.lib.interfaces;

import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LED {
    public static final CANdle m_candle = new CANdle(Constants.LED.CANdleID, "rio");
    public final int LedCount = 42;

    private static boolean backButtonPrev = false;
    public static boolean backButton = false;
    
    public static Animation m_toAnimate = null;

    public void CANdleSystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public static void periodic() {
        boolean backButtonCurr = RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.LED_BUTTON);
        backButton = !backButtonPrev && backButtonCurr;
        backButtonPrev = backButtonCurr;
    }
}
