// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import org.w3c.dom.html.HTMLTableColElement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake2.IntakeState.IntakeSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HopperToIntakeHandoff extends SequentialCommandGroup {
  /** Creates a new HopperToIntakeHandoff. */
  public HopperToIntakeHandoff(Hopper hopper, Intake2 intake, boolean retractAtEnd) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      intake.setStateCommand(IntakeSetpoint.EXTENDED_INTAKEING, true),
      hopper.setStateCommand(HopperState.HOPPER_TO_INTAKE),
      intake.setStateCommand(IntakeSetpoint.EXTENDED_EXTAKEING, false),
      new WaitUntilCommand(hopper::hasNote).andThen(new WaitCommand(0.1)).withTimeout(5.0),
      hopper.setStateCommand(HopperState.NEUTRAL),
      intake.setStateCommand(retractAtEnd ? IntakeSetpoint.RETRACTED_NEUTRAL : IntakeSetpoint.EXTENDED_NEUTRAL, false)
    );
  }
}
