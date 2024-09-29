// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.AmpBar.AmpBarState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmp extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  public ScoreAmp(Intake intake, AmpBar ampBar) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeState.AMP_NEUTRAL, true)
        .alongWith(ampBar.setStateCommand(AmpBarState.EXTENDED, true)),
      intake.setStateCommand(IntakeState.AMP_EXTAKEING, false),
      new WaitUntilCommand(() -> !intake.hasNote()).andThen(new WaitCommand(0.5)).withTimeout(2.0),
      intake.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false).alongWith(ampBar.setStateCommand(AmpBarState.RETRACTED, false))
    );
  }
}
