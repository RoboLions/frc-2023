package frc.robot.subsystems.LED;

import frc.robot.lib.interfaces.LED;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

public class ConeLEDState extends State {

    public void build(){
        transitions.add(new Transition(() -> {
            return LED.backButton;
        }, LEDStateMachine.cubeLEDState));   
    }

    @Override
    public void init() {
       
    }

    @Override
    public void execute() {
        LED.m_candle.setLEDs(255, 228, 0);
    }

    @Override
    public void exit() {
        LED.m_toAnimate = null;
        LED.m_candle.animate(LED.m_toAnimate);
    }
}
