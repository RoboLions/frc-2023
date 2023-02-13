// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.subsystems.arm.back.*;
import frc.robot.subsystems.arm.front.*;


/** Add your docs here. */
public class ArmStateMachine extends StateMachine {

    public static IdleState idleState = new IdleState();
    public static IntakeState intakeState = new IntakeState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static DropState dropState = new DropState();
    public static BHighPurple bHighPurple = new BHighPurple();
    public static BMidPurple bMidPurple = new BMidPurple();
    public static BHybrid bHybrid = new BHybrid();
    public static BHighYellow bHighYellow = new BHighYellow();
    public static BMidYellow bMidYellow = new BMidYellow();
    public static FHighPurple fHighPurple = new FHighPurple();
    public static FMidPurple fMidPurple = new FMidPurple();
    public static FHybrid fHybrid = new FHybrid();
    public static FHighYellow fHighYellow = new FHighYellow();
    public static FMidYellow fMidYellow = new FMidYellow();
    public static OpenState openState = new OpenState();
    public static PickupState pickupState = new PickupState();

    public ArmStateMachine() {

        idleState.build();
        intakeState.build();
        outtakeState.build();
        dropState.build();
        openState.build();
        pickupState.build();
        fMidYellow.build();
        fMidPurple.build();
        fHybrid.build();
        fHighPurple.build();
        fHighYellow.build();
        bMidYellow.build();
        bMidPurple.build();
        bHybrid.build();
        bHighPurple.build();
        bHighYellow.build();

        setCurrentState(idleState);
    }
}
