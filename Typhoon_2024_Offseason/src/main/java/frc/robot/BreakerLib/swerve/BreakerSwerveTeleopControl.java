// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import java.util.Optional;

import javax.print.DocPrintJob;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;

public class BreakerSwerveTeleopControl extends Command {
  /** Creates a new BreakerSwerveTeleopControl. */
  private BreakerSwerveDrivetrain drivetrain;
  private BreakerInputStream x, y, omega;
  private SwerveRequest.FieldCentric request;
  private HeadingCompensationConfig headingCompensationConfig;
  private PIDController pid;
  private Rotation2d headingSetpoint;
  /**
    Creates a BreakerSwerveTeleopControl command
    @param drivetrain
    @param x 
    @param y
    @param omega
  
  */

  public BreakerSwerveTeleopControl(BreakerSwerveDrivetrain drivetrain, BreakerInputStream x, BreakerInputStream y, BreakerInputStream omega, HeadingCompensationConfig headingCompensationConfig) {
    addRequirements(drivetrain);
    pid = new PIDController(headingCompensationConfig.pidConstants.kP, headingCompensationConfig.pidConstants.kI, headingCompensationConfig.pidConstants.kD);
    pid.enableContinuousInput(-Math.PI, Math.PI);
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.omega = omega;
    this.headingCompensationConfig = headingCompensationConfig;
    request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   headingSetpoint = drivetrain.getPigeon2().getRotation2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xImpt = x.get();
    double yImpt = y.get();
    double omegaImpt = omega.get();
    if (Math.hypot(xImpt, yImpt) >= headingCompensationConfig.minActiveLinearVelocity.in(Units.MetersPerSecond) && Math.abs(omegaImpt) > headingCompensationConfig.angularVelocityDeadband.in(Units.RadiansPerSecond)) {
      omegaImpt = pid.calculate(drivetrain.getPigeon2().getRotation2d().getRadians(), headingSetpoint.getRotations());
    } else {
      headingSetpoint = drivetrain.getPigeon2().getRotation2d();
    }
    request.withVelocityX(xImpt).withVelocityY(yImpt).withRotationalRate(omegaImpt);
    drivetrain.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public record HeadingCompensationConfig(Measure<Velocity<Distance>> minActiveLinearVelocity, Measure<Velocity<Angle>> angularVelocityDeadband, PIDConstants pidConstants) {
  }
}
