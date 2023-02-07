package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.lib.statemachine.Transition;

public class DrivetrainStateMachine extends StateMachine {

    public TeleopState teleopSwerve = new TeleopState();
    public FollowTag aprilTagState = new FollowTag();

    private static XboxController driverController = RobotMap.driverController;

    public DrivetrainStateMachine() {

        Supplier<Boolean> checkAButton = () -> {
            return driverController.getAButton();
        };

        Supplier<Boolean> checkBButton = () -> {
            return driverController.getBButton();
        };

        teleopSwerve.addTransition(new Transition(checkAButton, aprilTagState));
        aprilTagState.addTransition(new Transition(checkBButton, teleopSwerve));

        setCurrentState(teleopSwerve);
    }
    
}
