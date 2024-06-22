// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAndHold extends SequentialCommandGroup {
  /** Creates a new IntakeAndHold. */
  public IntakeAndHold(Intake intake, boolean retractIntakeOnSuccess) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, true),
      intake.setStateCommand(IntakeState.EXTENDED_INTAKEING, false),
      new WaitUntilCommand(intake::hasNote),
      new ConditionalCommand(intake.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false), intake.setStateCommand(IntakeState.EXTENDED_NEUTRAL, false),() -> retractIntakeOnSuccess)
    );
  }
}
