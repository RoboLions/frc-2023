package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.interfaces.SwerveModule;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

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

        addTransition(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.AUTO_BALANCE_BUTTON);
        }, DrivetrainStateMachine.balanceState));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {

        // for(SwerveModule mod : Swerve.mSwerveMods) {
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        // }
        
        translationVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        strafeVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        //     translationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        //     strafeVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        //     rotationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);
        // } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //     translationVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        //     strafeVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        //     rotationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);
        // }

        if (RobotMap.driverController.getRawButton(Constants.DriverControls.ZERO_GYRO)) {
            Translation2d current_coords = Swerve.swerveOdometry.getEstimatedPosition().getTranslation();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                RobotMap.swerve.resetOdometry(new Pose2d(current_coords, Rotation2d.fromDegrees(180.0)));
            }
            else {
                RobotMap.swerve.resetOdometry(new Pose2d(current_coords, Rotation2d.fromDegrees(0.0)));
            }
        }

        if (RobotMap.armStateMachine.getCurrentState() != ArmStateMachine.idleState) {
            RobotMap.swerve.drive(
                new Translation2d(translationVal, strafeVal).times(2.0), // Constants.SWERVE.MAX_SPEED), 
                rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY * 0.6,
                true, 
                true
            );
        } else {
            RobotMap.swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.SWERVE.MAX_SPEED), 
                rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY,
                true, 
                true
            );
        }
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
