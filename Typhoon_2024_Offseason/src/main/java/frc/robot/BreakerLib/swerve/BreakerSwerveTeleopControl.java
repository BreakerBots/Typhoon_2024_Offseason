// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;

public class BreakerSwerveTeleopControl extends Command {
  /** Creates a new BreakerSwerveTeleopControl. */
  private BreakerInputStream x, y, theta;
  private SwerveRequest.FieldCentric openHeadingRequest;
  private SwerveRequest.FieldCentricFacingAngle closedHeadingRequest;
  public BreakerSwerveTeleopControl(BreakerSwerveDrivetrain drivetrain, BreakerInputStream x, BreakerInputStream y, BreakerInputStream theta,  Optional<BreakerSwerveTeleopHeadingCompensationConfig> headingCompensationConfig) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public record BreakerSwerveTeleopHeadingCompensationConfig(Measure<Velocity<Distance>> minActiveLinearVelocity, Measure<Velocity<Angle>> angularVelocityDeadband) {
  }
}
