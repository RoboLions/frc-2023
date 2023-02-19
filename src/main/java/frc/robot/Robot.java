// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeExecutor;
import frc.robot.lib.auto.AutoModeSelector;
import frc.robot.lib.states.Swerve;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.claw.ClawStateMachine;
import frc.robot.subsystems.drive.DrivetrainStateMachine;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /* state machine instances */
  private DrivetrainStateMachine drivetrainStateMachine;
  private ArmStateMachine armStateMachine;
  private ClawStateMachine clawStateMachine;

  // auto instances
	private AutoModeExecutor autoModeExecutor;
	private AutoModeSelector autoModeSelector = new AutoModeSelector();

  public static Color detectedColor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.init();
    RobotMap.gyro.configFactoryDefault();
    RobotMap.shoulderMotor.configFactoryDefault();
    RobotMap.elbowMotor.configFactoryDefault();
    RobotMap.wristMotor.configFactoryDefault();
    RobotMap.clawMotor.configFactoryDefault();

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
    * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info. */
    Timer.delay(1.0);
    RobotMap.swerve.resetModulesToAbsolute();

    RobotMap.swerve.zeroGyro();

    SmartDashboard.putData("Field", RobotMap.Field2d);

    drivetrainStateMachine = new DrivetrainStateMachine();
    armStateMachine = new ArmStateMachine();
    clawStateMachine = new ClawStateMachine();
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
    drivetrainStateMachine.setNextState();
    armStateMachine.setNextState();
    clawStateMachine.setNextState();

    // update swerve pose estimator
    RobotMap.swerve.updatePoses();

    // see robot pose on Glass
    RobotMap.Field2d.setRobotPose(Swerve.swerveOdometry.getEstimatedPosition());

    // update color sensor on claw
    RobotMap.claw.updateDetectedColor();
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
    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
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
