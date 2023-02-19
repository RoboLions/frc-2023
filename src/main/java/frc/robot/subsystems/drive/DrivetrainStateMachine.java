package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.lib.statemachine.Transition;

public class DrivetrainStateMachine extends StateMachine {

    public static TeleopState teleopSwerve = new TeleopState();
    public static FollowTag aprilTagState = new FollowTag();
    public static BalanceState balanceState = new BalanceState();

    public DrivetrainStateMachine() {
        aprilTagState.build();
        balanceState.build();
        teleopSwerve.build();
        setCurrentState(teleopSwerve);
    }
    
}
