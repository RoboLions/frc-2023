package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.LED;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

public class LEDIdleState extends State {
    
    public void build() {  
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.CUBE_LED_BUTTON);
        }, LEDStateMachine.cubeLEDState));    

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.CONE_LED_BUTTON);
        }, LEDStateMachine.coneLEDState));   
    }
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {
        LED.m_candle.setLEDs(0, 0, 0);
    }

    @Override
    public void exit() {
        
    }

}