// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.State;
import frc.robot.lib.StateMachine;
import frc.robot.lib.Transition;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static XboxController manipulatorController;

  /* Instances */
  public static State AButton;
  private State BButton;
  private State XButton;
  private StateMachine stateMachine;

  private static Supplier<Boolean> checkAButton;
  private static Supplier<Boolean> checkBButton;
  private static Supplier<Boolean> checkXButton;

  private static Runnable initStatementA;
  private static Runnable executeStatementA;
  private static Runnable exitStatementA;

  private static Runnable initStatementB;
  private static Runnable executeStatementB;
  private static Runnable exitStatementB;

  private static Runnable initStatementX;
  private static Runnable executeStatementX;
  private static Runnable exitStatementX;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    manipulatorController = new XboxController(1);

    checkAButton = () -> {
      if (manipulatorController.getAButton() == false) {
        return false;
      }   
      return true;
    };

    checkBButton = () -> {
      if (manipulatorController.getBButton() == false) {
        return false;
      }   
      return true;
    };

    checkXButton = () -> {
      if (manipulatorController.getXButton() == false) {
        return false;
      }   
      return true;
    };

    initStatementA = () -> {
      System.out.println("I am entering state A");
    };

    executeStatementA = () -> {
      System.out.println("I am executing state A");
    };

    exitStatementA = () -> {
      System.out.println("I am exiting state A");
    };

    initStatementB = () -> {
      System.out.println("I am entering state B");
    };

    executeStatementB = () -> {
      System.out.println("I am executing state B");
    };

    exitStatementB = () -> {
      System.out.println("I am exiting state B");
    };

    initStatementX = () -> {
      System.out.println("I am entering state X");
    };

    executeStatementX = () -> {
      System.out.println("I am executing state X");
    };

    exitStatementX = () -> {
      System.out.println("I am exiting state X");
    };

    AButton = new State(Robot.initStatementA, Robot.executeStatementA, Robot.exitStatementA);
    AButton.addTransition(new Transition(checkBButton, BButton));

    BButton = new State(Robot.initStatementB, Robot.executeStatementB, Robot.exitStatementB);
    BButton.addTransition(new Transition(checkXButton, XButton));

    XButton = new State(Robot.initStatementX, Robot.executeStatementX, Robot.exitStatementX);
    XButton.addTransition(new Transition(checkAButton, AButton));

    stateMachine.setCurrentState(AButton);
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
    State currentState = stateMachine.getCurrentState();

    currentState.execute();

    State nextState = stateMachine.getNextState();
    if (nextState != currentState) {
      currentState.exit();
      nextState.init();
      stateMachine.setCurrentState(nextState);
    }
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
