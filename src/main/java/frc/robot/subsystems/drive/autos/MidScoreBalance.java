package frc.robot.subsystems.drive.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.ConditionAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.lib.interfaces.Arm;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.drive.DrivetrainStateMachine;

/** Simple mid score, 1 cone high, then balance on charging station */
public class MidScoreBalance extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveToChargeStation;

    Pose2d initialHolonomicPose;

    public MidScoreBalance() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        
        // transform trajectory depending on alliance we are on
        PathPlannerTrajectory botMidScore = PathPlanner.loadPath("Mid Score + Balance", new PathConstraints(2.5, 2.0));
        botMidScore = PathPlannerTrajectory.transformTrajectoryForAlliance(botMidScore, DriverStation.getAlliance());
        
        initialHolonomicPose = botMidScore.getInitialHolonomicPose();

        driveToChargeStation = new TrajectoryAction(
            botMidScore, 
            RobotMap.swerve::getPose,
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        System.out.println("Running mid score with balance auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // close the claw
        // runAction(new LambdaAction(() -> RobotMap.clawStateMachine.maintainState(ClawStateMachine.closingState)));
        
        // // position arm to score high
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // // wait for arm to arrive in position
        // runAction(new ConditionAction(() -> {
        //   return Arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        // }));

        // // then, score the piece
        // runAction(new LambdaAction(() -> RobotMap.clawStateMachine.setCurrentState(ClawStateMachine.closedState)));
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // wait for the piece to be scored which means the arm is in idle
        runAction(new ConditionAction(() -> {
            return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.idleState;
        }));

        // drive onto the charge station
        runAction(driveToChargeStation);

        // switch drivetrain to balance state
        runAction(new LambdaAction(() -> RobotMap.drivetrainStateMachine.setCurrentState(DrivetrainStateMachine.balanceState)));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return initialHolonomicPose;
    }
}
