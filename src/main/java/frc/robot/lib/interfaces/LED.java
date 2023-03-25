package frc.robot.lib.interfaces;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

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
