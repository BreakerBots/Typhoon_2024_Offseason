// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeForShooter extends SequentialCommandGroup {
  /** Creates a new IntakeForShooter. */
  public IntakeForShooter(Intake intake, Hopper hopper, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true)
        .alongWith(shooter.setStateCommand(ShooterConstants.INTAKE_HANDOFF_STATE, true)),
      intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, false)
        .alongWith(hopper.setStateCommand(HopperState.INTAKE_TO_HOPPER)),
      new WaitUntilCommand(hopper::hasNote),
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false)
        .alongWith(hopper.setStateCommand(HopperState.NEUTRAL))
    );
  }
}
