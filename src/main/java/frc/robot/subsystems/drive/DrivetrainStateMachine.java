package frc.robot.subsystems.drive;

import frc.robot.lib.statemachine.StateMachine;

public class DrivetrainStateMachine extends StateMachine {

    public static TeleopState teleopSwerve = new TeleopState();
    public static FollowTag followTag = new FollowTag();
    public static BalanceState balanceState = new BalanceState();

    public DrivetrainStateMachine() {
        followTag.build();
        balanceState.build();
        teleopSwerve.build();
        setCurrentState(teleopSwerve);
    }
    
}
