package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** State for balancing on the charging station autonomously */
public class BalanceState extends State {

    public double pitchP = Constants.BALANCE_PITCH_PID.P;
    public double pitchI = Constants.BALANCE_PITCH_PID.D;
    public double pitchD = Constants.BALANCE_PITCH_PID.I;

    public double rollP = Constants.BALANCE_ROLL_PID.P;
    public double rollI = Constants.BALANCE_ROLL_PID.I;
    public double rollD = Constants.BALANCE_ROLL_PID.D;

    // public static RoboLionsPID rollPID = new RoboLionsPID();
    // public static RoboLionsPID pitchPID = new RoboLionsPID();

    private PIDController pitchController = new PIDController(
        pitchP, pitchI, pitchD
    );

    private PIDController rollController = new PIDController(
        rollP, rollI, rollD
    );

    @Override
    public void build(){
        // if driver joysticks are engaged, transition to teleop state
        addTransition(new Transition(() -> {
            return Math.abs(RobotMap.driverController.getLeftX()) > Constants.STICK_DEADBAND || 
                Math.abs(RobotMap.driverController.getLeftY()) > Constants.STICK_DEADBAND || 
                Math.abs(RobotMap.driverController.getRightX()) > Constants.STICK_DEADBAND ||
                Math.abs(RobotMap.driverController.getRightY()) > Constants.STICK_DEADBAND;
        }, DrivetrainStateMachine.teleopSwerve));
    }

    @Override
    public void init() {
        // rollPID.initialize(
        //     rollD,
        //     rollI,
        //     rollD,
        //     0, 
        //     5, 
        //     Constants.SWERVE.MAX_SPEED
        // );
        // rollPID.enableDeadBand = true;
        // rollPID.enableCage = false;

        // pitchPID.initialize(
        //     pitchD,
        //     pitchI,
        //     pitchD,
        //     0, 
        //     5, 
        //     Constants.SWERVE.MAX_SPEED
        // );
        // pitchPID.enableDeadBand = true;
        // pitchPID.enableCage = false;
    }

    @Override
    public void execute() {

        double rollCommand = rollController.calculate(Swerve.getRoll(), 0.0);
        double pitchCommand = pitchController.calculate(Swerve.getPitch(), 0.0);

        if (Math.abs(Swerve.getRoll()) < Constants.BALANCE_PITCH_PID.DEADBAND) {
            rollCommand = 0.0;
        }

        if (Math.abs(Swerve.getPitch()) < Constants.BALANCE_ROLL_PID.DEADBAND) {
            pitchCommand = 0.0;
        }

        RobotMap.swerve.drive(
            new Translation2d(
                // rollPID.execute(0.0, RobotMap.gyro.getRoll()),
                // pitchPID.execute(0.0, RobotMap.gyro.getPitch())
                -rollCommand,
                -pitchCommand
            ).times(1.0), // Constants.SWERVE.MAX_SPEED), 
            0.0, //RobotMap.swerve.getPose().getRotation().getDegrees(), 
            false, 
            true
        );
    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0.0, 0.0).times(Constants.SWERVE.MAX_SPEED), 
            0.0,
            true,
            true
        );
    }
}