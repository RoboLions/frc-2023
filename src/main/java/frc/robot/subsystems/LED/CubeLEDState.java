package frc.robot.subsystems.LED;
import frc.robot.lib.interfaces.LED;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;


public class CubeLEDState extends State {
    
    public void build(){
        transitions.add(new Transition(() -> {
            return LED.backButton;
        }, LEDStateMachine.coneLEDState));
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        LED.m_candle.setLEDs(174, 0, 255);
    }

    @Override
    public void exit() {
        LED.m_toAnimate = null;        
    }

}