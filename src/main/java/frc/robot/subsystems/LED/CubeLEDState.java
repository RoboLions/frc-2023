package frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.LED;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class CubeLEDState extends State {
    
    public void build(){
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.CONE_LED_BUTTON);
        }, LEDStateMachine.LEDIdleState));    
    }

    @Override
    public void init(State prevState) {

    }

    @Override
    public void execute() {
    LED.m_candle.setLEDs(174, 0, 255);
    }

    @Override
    public void exit(State nextState) {
        LED.m_toAnimate = null;        
    }

}