package frc.robot.ExampleSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.lib.StateMachine;
import frc.robot.lib.Transition;

public class ExampleSubsystemStateMachine extends StateMachine {

    public StateA AButton = new StateA();
    public StateB BButton = new StateB();
    public StateX XButton = new StateX();

    private static XboxController manipulatorController = Robot.manipulatorController;
    
    public ExampleSubsystemStateMachine() {

        Supplier<Boolean> checkAButton = () -> {
            return manipulatorController.getAButton();
        };

        Supplier<Boolean> checkBButton = () -> {
            return manipulatorController.getBButton();
        };

        Supplier<Boolean> checkXButton = () -> {
            return manipulatorController.getXButton();
        };

        AButton.addTransition(new Transition(checkBButton, BButton));
        BButton.addTransition(new Transition(checkXButton, XButton));
        BButton.addTransition(new Transition(checkAButton, AButton));
        XButton.addTransition(new Transition(checkAButton, AButton));

        setCurrentState(AButton);
    }
}
