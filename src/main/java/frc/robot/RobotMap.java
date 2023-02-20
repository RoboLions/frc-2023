package frc.robot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.interfaces.Arm;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.claw.ClawStateMachine;
import frc.robot.subsystems.drive.DrivetrainStateMachine;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotMap {

    /* state machine instances */
    public static DrivetrainStateMachine drivetrainStateMachine;
    public static ArmStateMachine armStateMachine;
    public static ClawStateMachine clawStateMachine;

    /* Motor instances */
    public static WPI_Pigeon2 gyro;
    public static WPI_TalonFX elbowMotor;
    public static WPI_TalonFX shoulderMotor;
    public static WPI_TalonFX wristMotor;
    public static VictorSPX clawMotor;
    // TODO: tbd intake motors

    /* Sensor instances */
    public static ColorSensorV3 clawColorSensor;

    /* Smart Dashboard Instances */
    public static Field2d Field2d;

    /* Interface instances */
    public static Swerve swerve; 
    public static Arm arm;
    public static Claw claw;

    /* Xbox controllers */
    public static XboxController manipulatorController;
    public static XboxController driverController;

    /* Claw open and close requests */
    public static boolean openRequest = false;
    public static boolean closeRequest = false;

    public static void init() {
        
        gyro = new WPI_Pigeon2(Constants.CAN_IDS.PIDGEON);
        shoulderMotor = new WPI_TalonFX(Constants.CAN_IDS.SHOULDER_MOTOR);
        elbowMotor = new WPI_TalonFX(Constants.CAN_IDS.ELBOW_MOTOR);
        wristMotor = new WPI_TalonFX(Constants.CAN_IDS.WRIST_MOTOR);
        clawMotor = new VictorSPX(Constants.CAN_IDS.CLAW_MOTOR);
        clawColorSensor = new ColorSensorV3(Constants.PORTS.COLOR_SENSOR);

        gyro.configFactoryDefault();
        shoulderMotor.configFactoryDefault();
        elbowMotor.configFactoryDefault();
        wristMotor.configFactoryDefault();
        clawMotor.configFactoryDefault();
        
        swerve = new Swerve();
        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
        * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info. */
        Timer.delay(1.0);
        swerve.resetModulesToAbsolute();
        swerve.zeroGyro();

        manipulatorController = new XboxController(1);
        driverController = new XboxController(0);
        
        arm = new Arm();
        claw = new Claw();
        Field2d = new Field2d();

        drivetrainStateMachine = new DrivetrainStateMachine();
        armStateMachine = new ArmStateMachine();
        clawStateMachine = new ClawStateMachine();
    }
}
