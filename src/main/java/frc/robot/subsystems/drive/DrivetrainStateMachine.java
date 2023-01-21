package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.lib.StateMachine;
import frc.robot.lib.Transition;

public class DrivetrainStateMachine extends StateMachine {

    public TeleopState teleopSwerve = new TeleopState();

    private static XboxController driverController = Robot.driverController;

    public DrivetrainStateMachine() {
        setCurrentState(teleopSwerve);
    }
    
}
