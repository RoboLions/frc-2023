// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.Constants;

/** Add your docs here. */
public class OpenState extends State {
    
    @Override
    public void build() {
        // close on a cube if "close request" and color sensor == purple
        transitions.add(new Transition(() -> {
            return (RobotMap.claw.getColor() == Constants.CLAW.CUBE_COLOR) && !Claw.openRequest;
        }, ClawStateMachine.closingCube));
        //TODO: Figure out which state to transition to based on request
        transitions.add(new Transition(() -> {
            return Claw.closeRequest;
        }, ClawStateMachine.closingCube));
    
        // close on a cone if "close request" and color sensor == yellow
        transitions.add(new Transition(() -> {
            return (RobotMap.claw.getColor() == Constants.CLAW.CONE_COLOR) && !Claw.openRequest;
        }, ClawStateMachine.closingCone));
        
        
    }

    @Override
    public void init() {
        RobotMap.claw.setClawOpen();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        // set closeRequest to false
        Claw.closeRequest = false;
    }
}
