package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.interfaces.Arm;
import frc.robot.lib.interfaces.Intake;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.interfaces.LED;
import frc.robot.subsystems.LED.LEDStateMachine;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.drive.DrivetrainStateMachine;
import frc.robot.subsystems.intake.IntakeStateMachine;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotMap {

    /* state machine instances */
    public static DrivetrainStateMachine drivetrainStateMachine;
    public static ArmStateMachine armStateMachine;
    public static IntakeStateMachine intakeStateMachine;
    public static LEDStateMachine ledStateMachine;

    /* Motor instances */
    public static WPI_Pigeon2 gyro;
    public static WPI_TalonFX leftElbowMotor;
    public static WPI_TalonFX rightElbowMotor;
    public static WPI_TalonFX leftShoulderMotor;
    public static WPI_TalonFX rightShoulderMotor;
    public static VictorSPX intakeMotor;

    /* Smart Dashboard Instances */
    public static Field2d Field2d;

    /* Interface instances */
    public static Swerve swerve; 
    public static Arm arm;
    public static Intake intake;
    public static LED led;

    /* Xbox controllers */
    public static XboxController manipulatorController;
    public static XboxController driverController;

    public static void init() {
        
        gyro = new WPI_Pigeon2(Constants.CAN_IDS.PIDGEON, "Swerve");
        leftShoulderMotor = new WPI_TalonFX(Constants.CAN_IDS.LEFT_SHOULDER_MOTOR);
        rightShoulderMotor = new WPI_TalonFX(Constants.CAN_IDS.RIGHT_SHOULDER_MOTOR);
        leftElbowMotor = new WPI_TalonFX(Constants.CAN_IDS.LEFT_ELBOW_MOTOR);
        rightElbowMotor = new WPI_TalonFX(Constants.CAN_IDS.RIGHT_ELBOW_MOTOR);
        intakeMotor = new VictorSPX(Constants.CAN_IDS.INTAKE_MOTOR);

        gyro.configFactoryDefault();
        leftShoulderMotor.configFactoryDefault();
        rightShoulderMotor.configFactoryDefault();
        leftElbowMotor.configFactoryDefault();
        rightElbowMotor.configFactoryDefault();
        intakeMotor.configFactoryDefault();
        
        swerve = new Swerve();
        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info. */
        Timer.delay(1.0);
        swerve.resetModulesToAbsolute();
        swerve.zeroGyro();

        manipulatorController = new XboxController(1);
        driverController = new XboxController(0);
        
        arm = new Arm();
        intake = new Intake();
        Field2d = new Field2d();
        led = new LED();

        drivetrainStateMachine = new DrivetrainStateMachine();
        armStateMachine = new ArmStateMachine();
        intakeStateMachine = new IntakeStateMachine();
        ledStateMachine = new LEDStateMachine();
    }
}
