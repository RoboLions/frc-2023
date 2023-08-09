// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Elevator;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorModule {
    private final String motorName;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    public ElevatorModule(ElevatorIO io, String motorName){
        this.io = io;
        this.motorName = motorName;
    }
    public void periodic(){
        io.updateInputs(inputs);
        Logger.getInstance().processInputs(motorName + " Elevator", inputs);
    }
}
