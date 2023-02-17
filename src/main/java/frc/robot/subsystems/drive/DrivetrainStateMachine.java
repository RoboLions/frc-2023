package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.lib.statemachine.Transition;

public class DrivetrainStateMachine extends StateMachine {

    public static BalanceState balanceState = new BalanceState();
    public static FollowTag aprilTag = new FollowTag();
    public static ScoreState scoreState = new ScoreState();
    public static TeleopState teleopState = new TeleopState();

    private static XboxController driverController = RobotMap.driverController;

    public DrivetrainStateMachine() {
        
        aprilTag.build();
        balanceState.build();
        scoreState.build();
        teleopState.build();
    
    } 
    
}
