package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.SwerveModule;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.lib.states.Swerve;

public class TeleopState extends State {
    
    double translationVal;
    double strafeVal;
    double rotationVal;

    public TeleopState() {}

    @Override
    public void build() {
        addTransition(new Transition(() -> {
            return RobotMap.driverController.getRawButtonPressed(Constants.DriverButtons.AUTO_ALIGN_BUTTON);
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
        translationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverButtons.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        strafeVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverButtons.STRAFE_VAL), Constants.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(RobotMap.driverController.getRawAxis(Constants.DriverButtons.ROTATION_VAL), Constants.STICK_DEADBAND);

        // if left trigger pressed, zero gyro
        if (RobotMap.driverController.getRawAxis(Constants.DriverButtons.ZERO_GYRO) > Constants.STICK_DEADBAND) {
            RobotMap.swerve.zeroGyro();
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );

    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }
}
