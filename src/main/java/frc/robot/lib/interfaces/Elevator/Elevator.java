// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/** Add your docs here. */
public class Elevator {
    private TrapezoidProfile profile;
    private TrapezoidProfile.Constraints constraints = Constants.Elevator.constraints;
  

    public ElevatorModule normalElevatorMotor;
    public ElevatorModule reverseElevatorMotor;


    public Elevator(ElevatorIO normalElevatorIO, ElevatorIO reverseElevatorIO){
        normalElevatorMotor = new ElevatorModule(normalElevatorIO, "Normal");
        reverseElevatorMotor = new ElevatorModule(reverseElevatorIO, "Reverse");
    }
    public void setPointDrive(double goal){
        profile = new TrapezoidProfile(constraints, 
        new TrapezoidProfile.State(goal, 0), 
        new TrapezoidProfile.State(normalElevatorMotor.inputs.elevatorSensorPosition, normalElevatorMotor.inputs.elevatorSensorvelocity));
        var setpoint = profile.calculate(0.02);
        normalElevatorMotor.io.setMotorPositionOutput(setpoint.position);
    }
    public void manualDrive(double translationVal){
        profile = new TrapezoidProfile(constraints, 
        new TrapezoidProfile.State(translationVal, 0));
        var setpoint = profile.calculate(0.02);
        normalElevatorMotor.io.setMotorPercentOutput(setpoint.position);

    }
    public void periodic(){
        normalElevatorMotor.periodic();
        reverseElevatorMotor.periodic();
    }
}
