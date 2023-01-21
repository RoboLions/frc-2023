package frc.robot.ExampleSubsystem;
import frc.robot.lib.StateMachine;

public class ExampleSubsystemStateMachine extends StateMachine {
    
    public ExampleSubsystemStateMachine() {
        super();
        setCurrentState(StateA.getInstance());
    }
}
