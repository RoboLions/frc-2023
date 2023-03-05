// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class SubstationIntakeState extends State {

    private Timer substationIntakeTimer = new Timer();
    
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

        // TODO: transition back to idle when we have a piece
        // transitions.add(new Transition(() -> {
        //     return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCone || RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCube;
        // }, ArmStateMachine.elbowIdleState));

        // if we hold a cone or cube
        // transitions.add(new Transition(() -> {
        //     return RobotMap.claw.getColor() == Constants.CLAW.CONE_COLOR || RobotMap.claw.getColor() == Constants.CLAW.CUBE_COLOR;
        // }, ArmStateMachine.elbowIdleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.SUBSTATION_INTAKE.SHOULDER_POSITION, 
            Constants.SUBSTATION_INTAKE.ELBOW_POSITION
        );

        substationIntakeTimer.start();
    }

    @Override
    public void execute() {

        // TODO: open request for claw somehow
    }

    @Override
    public void exit() {
        substationIntakeTimer.stop();
        substationIntakeTimer.reset();
        Claw.requestClawClosed();
    }
}
