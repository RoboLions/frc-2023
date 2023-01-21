package frc.robot.ExampleSubsystem;

import frc.robot.OI;
import frc.robot.lib.State;

public final class StateX extends State {

    private StateX() {
        super();
        addTransition(() -> {
            return OI.manipulatorController.getAButton();
        }, StateA.getInstance());
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
