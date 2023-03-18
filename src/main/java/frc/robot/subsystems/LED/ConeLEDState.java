package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.LED;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

public class ConeLEDState extends State {

    public void build(){
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.CUBE_LED_BUTTON);
        }, LEDStateMachine.LEDIdleState));    
    }

    @Override
    public void init() {
       //LED.m_toAnimate = new StrobeAnimation(255, 228, 0, 0, 0.5, 68);
       //LED.m_toAnimate = new LarsonAnimation(255, 228, 0, 0, 0.5, 68, LarsonAnimation.BounceMode.Front, 2);
       //LED.m_toAnimate = new TwinkleAnimation(255, 228, 0, 0, 0.5, 68, TwinkleAnimation.TwinklePercent.Percent30);
    }

    @Override
    public void execute() {
        LED.m_candle.setLEDs(255, 228, 0);
        //LED.m_candle.animate(LED.m_toAnimate);
    }

    @Override
    public void exit() {
        LED.m_toAnimate = null;
        LED.m_candle.animate(LED.m_toAnimate);
    }

}
