// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Swerve;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs{
       public double yaw;
       public double pitch;
       public double roll;
    }
    public default void updateInputs(GyroIOInputs inputs) {}
    public default void zeroGyro(){}
}
