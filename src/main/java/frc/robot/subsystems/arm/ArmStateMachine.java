// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.back.BHighPurple;
import frc.robot.subsystems.arm.back.BHighYellow;
import frc.robot.subsystems.arm.back.BHybrid;
import frc.robot.subsystems.arm.back.BMidPurple;
import frc.robot.subsystems.arm.back.BMidYellow;
import frc.robot.subsystems.arm.front.FHighPurple;
import frc.robot.subsystems.arm.front.FHighYellow;
import frc.robot.subsystems.arm.front.FHybrid;
import frc.robot.subsystems.arm.front.FMidPurple;
import frc.robot.subsystems.arm.front.FMidYellow;

/** Add your docs here. */
public class ArmStateMachine extends StateMachine {

    public static IdleState idleState = new IdleState();
    public static IntakeState intakeState = new IntakeState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static DropState dropState = new DropState();
    public static OpenState openState = new OpenState();
    public static PickupState pickupState = new PickupState();
    public static FMidYellow frontMidYellow = new FMidYellow();
    public static FMidPurple frontMidPurple = new FMidPurple();
    public static FHybrid frontHybrid = new FHybrid();
    public static FHighPurple frontHighPurple = new FHighPurple();
    public static FHighYellow frontHighYellow = new FHighYellow();
    public static BHybrid backHybrid = new BHybrid();
    public static BHighPurple backHighPurple = new BHighPurple();
    public static BMidPurple backMidPurple = new BMidPurple();
    public static BMidYellow backMidYellow = new BMidYellow();
    public static BHighYellow backHighYellow = new BHighYellow();

    public ArmStateMachine() {

        idleState.build();
        intakeState.build();
        outtakeState.build();
        dropState.build();
        openState.build();
        pickupState.build();
        frontMidYellow.build();
        frontMidPurple.build();
        frontHybrid.build();
        frontHighPurple.build();
        frontHighYellow.build();
        backMidYellow.build();
        backMidPurple.build();
        backHybrid.build();
        backHighPurple.build();
        backHighYellow.build();

        setCurrentState(idleState);
    }
}
