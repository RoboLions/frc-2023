// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.subsystems.arm.back.BHighPurple;
import frc.robot.subsystems.arm.back.BMidPurple;
import frc.robot.subsystems.arm.back.BHybrid;
import frc.robot.subsystems.arm.back.BHighYellow;
import frc.robot.subsystems.arm.back.BMidYellow;
import frc.robot.subsystems.arm.front.FHighPurple;
import frc.robot.subsystems.arm.front.FMidPurple;
import frc.robot.subsystems.arm.front.FHybrid;
import frc.robot.subsystems.arm.front.FHighYellow;
import frc.robot.subsystems.arm.front.FMidYellow;

public class ArmStateMachine extends StateMachine {
    /**
     * BUTTON CONFIGURATIONS:
     *  in all states - 
     *      B = idle state
     *  in all scoring states - 
     *      LEFT BUMPER = open claw button (this is currently in the final-claw branch of this repo)
     *      Y = higher level
     *      A = lower level 
     *      (in high state, press Y for mid and A for hybrid)
     *      (in mid state, press Y for high and A for hybrid)
     *      (in hybrid state, press Y for high and A for mid)
     *  in idle arm state -
     *      A = intake at substation
     *      X = high node
     *      Y = mid node
     *      B = hybrid node
     *      LEFT BUMPER = manual move
     */

    public static IdleState idleState = new IdleState();
    public static IntakeState intakeState = new IntakeState();
    public static ManualMoveState manualMoveState = new ManualMoveState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static PickupState pickupState = new PickupState();
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

    public ArmStateMachine() {

        idleState.build();
        intakeState.build();
        manualMoveState.build();
        outtakeState.build();
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
