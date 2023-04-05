// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeExecutor;
import frc.robot.lib.auto.AutoModeSelector;
import frc.robot.lib.interfaces.Intake;
import frc.robot.lib.interfaces.LED;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.interfaces.SwerveModule;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.drive.DrivetrainStateMachine;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // auto instances
	private AutoModeExecutor autoModeExecutor;
	private AutoModeSelector autoModeSelector = new AutoModeSelector();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.init();
    Swerve.zeroPitch();
    Swerve.zeroRoll();
    RobotMap.arm.resetEncoders();
    SmartDashboard.putData("Field", RobotMap.Field2d);
    RobotMap.intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /* state machines always execute current state and check for next state */
    RobotMap.drivetrainStateMachine.setNextState();
    RobotMap.intakeStateMachine.setNextState();
    RobotMap.armStateMachine.setNextState();
    RobotMap.ledStateMachine.setNextState();

    // update swerve pose estimator
    RobotMap.swerve.updatePoses();
    Swerve.periodic();
    LED.periodic();

    // see robot pose on Glass
    // RobotMap.Field2d.setRobotPose(Swerve.swerveOdometry.getEstimatedPosition());
    FieldObject2d currentPoseDisplay = RobotMap.Field2d.getObject("currentPose");
    currentPoseDisplay.setPose(Swerve.swerveOdometry.getEstimatedPosition());
    
    // SmartDashboard.putNumber("Integrated Encoder Shoulder (L)", RobotMap.leftShoulderMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Integrated Encoder Elbow (L)", RobotMap.leftElbowMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Integrated Encoder Shoulder (R)", RobotMap.rightShoulderMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Integrated Encoder Elbow (R)", RobotMap.rightElbowMotor.getSelectedSensorPosition());

    // // SmartDashboard.putNumber("Shoulder L Setpoint", RobotMap.leftShoulderMotor.getClosedLoopTarget());
    // // SmartDashboard.putNumber("Shoulder R Setpoint", RobotMap.rightShoulderMotor.getClosedLoopTarget());
    // SmartDashboard.putNumber("Elbow L Setpoint", RobotMap.leftElbowMotor.getClosedLoopTarget());
    // SmartDashboard.putNumber("Elbow R Setpoint", RobotMap.rightElbowMotor.getClosedLoopTarget());
    
    // SmartDashboard.putString("Current arm state", RobotMap.armStateMachine.getCurrentState().toString().replace("frc.robot.subsystems.arm.", ""));
    // SmartDashboard.putString("Current claw state", RobotMap.intakeStateMachine.getCurrentState().toString().replace("frc.robot.subsystems.claw.", ""));
    // // SmartDashboard.putString("Current drivetrain state", RobotMap.drivetrainStateMachine.getCurrentState().toString().replace("frc.robot.subsystems.drive.", ""));

    // SmartDashboard.putNumber("Error Shoulder", RobotMap.leftShoulderMotor.getClosedLoopError());
    // SmartDashboard.putNumber("Error Elbow", RobotMap.leftElbowMotor.getClosedLoopError());
    
    // Color read_color = RobotMap.intake.getColor();
    // //System.out.println(read_color);
    // SmartDashboard.putString("Detected HEX code", read_color != null ? read_color.toString() : "");
    //SmartDashboard.putNumber("Claw set power", RobotMap.intakeMotor.getMotorOutputPercent());
    // SmartDashboard.putNumber("Claw encoder", RobotMap.intakeEncoder.get());

    // SmartDashboard.putNumber("Roll", Swerve.getRoll());
    // SmartDashboard.putNumber("Pitch", Swerve.getPitch());

    //SmartDashboard.putNumber("Claw moto", kDefaultPeriod)

    // for(SwerveModule mod : Swerve.mSwerveMods) {
    //   SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    //   SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
    //   SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    // }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Optional<AutoModeBase> autoMode = autoModeSelector.getAutoMode();
    if (autoMode.isPresent()) {
      RobotMap.swerve.resetOdometry(autoMode.get().getStartingPose());
    }

		autoModeExecutor.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    RobotMap.drivetrainStateMachine.setCurrentState(DrivetrainStateMachine.teleopSwerve);
    // RobotMap.armStateMachine.setCurrentState(ArmStateMachine.idleState);
    
    RobotMap.leftShoulderMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.rightShoulderMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.leftElbowMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.rightElbowMotor.setNeutralMode(NeutralMode.Brake);

    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println(
    //   Swerve.mSwerveMods[0].mAngleMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[1].mAngleMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[2].mAngleMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[3].mAngleMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[0].mDriveMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[1].mDriveMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[2].mDriveMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[3].mDriveMotor.getMotorOutputVoltage() + "," +
    //   Swerve.mSwerveMods[0].mAngleMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[1].mAngleMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[2].mAngleMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[3].mAngleMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[0].mDriveMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[1].mDriveMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[2].mDriveMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[3].mDriveMotor.getStatorCurrent() + "," + 
    //   Swerve.mSwerveMods[0].mAngleMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[1].mAngleMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[2].mAngleMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[3].mAngleMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[0].mDriveMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[1].mDriveMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[2].mDriveMotor.getSelectedSensorVelocity() + "," + 
    //   Swerve.mSwerveMods[3].mDriveMotor.getSelectedSensorVelocity()
    // );
    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
    RobotMap.leftShoulderMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.rightShoulderMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.leftElbowMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.rightElbowMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.intakeMotor.setNeutralMode(NeutralMode.Coast);
    
    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }

    // Reset all auto mode state.
    autoModeSelector.reset();
    autoModeSelector.updateModeCreator();
    autoModeExecutor = new AutoModeExecutor();
  }

  @Override
  public void disabledPeriodic() {
    autoModeSelector.updateModeCreator();
    Optional<AutoModeBase> autoMode = autoModeSelector.getAutoMode();
    if (autoMode.isPresent() && autoMode.get() != autoModeExecutor.getAutoMode()) {
      System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
      autoModeExecutor.setAutoMode(autoMode.get());
    }
  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
