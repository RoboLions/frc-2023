package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.interfaces.SwerveModule;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

public class TeleopState extends State {
    
    double translationVal;
    double strafeVal;
    double rotationVal;

    public TeleopState() {}

    @Override
    public void build() {
        addTransition(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.AUTO_ALIGN_BUTTON);
        }, DrivetrainStateMachine.followTag));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {

        for(SwerveModule mod : Swerve.mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // invert because Xbox controllers return negative values when we push forward
        translationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        strafeVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);

        // if y button pressed, zero gyro
        if (RobotMap.driverController.getRawButton(Constants.DriverControls.ZERO_GYRO)) {
            RobotMap.swerve.zeroGyro();
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(1.0), // Constants.SWERVE.MAX_SPEED), 
            rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY * 0.5, 
            true, 
            true
        );

    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0, 0).times(Constants.SWERVE.MAX_SPEED), 
            0 * Constants.SWERVE.MAX_ANGULAR_VELOCITY, 
            true, 
            true
        );
    }
}
