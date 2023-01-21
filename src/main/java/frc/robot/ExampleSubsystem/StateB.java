package frc.robot.ExampleSubsystem;

import frc.robot.OI;
import frc.robot.lib.State;

public final class StateB extends State {

    private StateB() {
        super();
        addTransition(() -> {
            return OI.manipulatorController.getXButton();
        }, StateX.getInstance());
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }
    
}
