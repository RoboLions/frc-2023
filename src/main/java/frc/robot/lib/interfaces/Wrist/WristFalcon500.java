// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Wrist;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class WristFalcon500 implements WristIO{
    public TalonFX wristMotor;
    public WristFalcon500(int WristMotorID){
        wristMotor = new TalonFX(WristMotorID);
    }

    public void updateInputs(WristIOInputs input){
        input.WristPosition = wristMotor.getSelectedSensorPosition();
        input.WristVelocity = wristMotor.getSelectedSensorVelocity();
        input.WristOutput = wristMotor.getMotorOutputPercent();
    }
}
