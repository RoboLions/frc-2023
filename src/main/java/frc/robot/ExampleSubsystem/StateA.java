package frc.robot.ExampleSubsystem;

import frc.robot.lib.State;
import frc.robot.OI;

public final class StateA extends State {

    private StateA() {
        super();
        addTransition(() -> {
            return OI.manipulatorController.getBButton();
        }, StateB.getInstance());
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
