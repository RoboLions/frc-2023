// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Wrist;

/** Add your docs here. */
public class Wrist {
    public WristIO io;
    public WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    public Wrist(WristIO io){
        this.io = io;
    }
    public void periodic(){
        io.updateInputs(inputs);
    }
}