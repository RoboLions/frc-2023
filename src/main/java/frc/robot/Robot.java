// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/*added with advantage kit library, works just like timed robot 
It does not support adding extra periodic functions.The method setUseTiming allows the user code to 
disable periodic timing and run cycles as fast as possible during replay. The timestamp read by methods
like Timer.getFPGATimstamp() will still match the timestamp from the real robot.
---------------------------------------------------------------------------- */
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
/*----------------------------------------------------------- */

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.states.Swerve;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.drive.DrivetrainStateMachine;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /* state machine instances */
  private DrivetrainStateMachine drivetrainStateMachine;
  private ArmStateMachine armStateMachine;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*advantagekit init code */
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

if (isReal()) {
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
    Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
} else {
    setUseTiming(false); // Run as fast as possible
    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
}

Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.


    RobotMap.init();
    RobotMap.gyro.configFactoryDefault();
    RobotMap.armFirstStage.configFactoryDefault();
    RobotMap.armSecondStage.configFactoryDefault();
    RobotMap.wrist.configFactoryDefault();
    RobotMap.intakeRoller.configFactoryDefault();
    RobotMap.claw.configFactoryDefault();
    RobotMap.swerve.zeroGyro();

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
    * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info. */
    Timer.delay(1.0);
    RobotMap.swerve.resetModulesToAbsolute();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putData("Field", RobotMap.Field2d);

    drivetrainStateMachine = new DrivetrainStateMachine();
    armStateMachine = new ArmStateMachine();
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

    RobotMap.swerve.updatePoses();
    RobotMap.Field2d.setRobotPose(Swerve.swerveOdometry.getEstimatedPosition());
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
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
