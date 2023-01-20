package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    
    public static XboxController manipulatorController = Robot.manipulatorController;

    public boolean getAButton() {
        return manipulatorController.getAButton();
    }

    public boolean getBButton() {
        return manipulatorController.getBButton();
    }

    public boolean getXButton() {
        return manipulatorController.getXButton();
    }
}
