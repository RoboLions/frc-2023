// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Wrist;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface WristIO  {
    @AutoLog
    public class WristIOInputs {
        public double WristPosition;
        public double WristVelocity;
        public double WristOutput;
    }
    public default void updateInputs(WristIOInputs inputs){}
}
