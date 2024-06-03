// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.InputStream;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {
  private final BreakerXboxController controller = new BreakerXboxController(0);
  private final Intake intake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();
  private BreakerInputStream driverX, driverY, driverOmega;
  private SwerveRequest.FieldCentric teleopSwerveRequest;
  public RobotContainer() {
    configureControls();
    boolean fileOnly = false;
    boolean lazyLogging = false;
    Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
  }

  private void configureControls() {
    driverX = controller.getLeftThumbstick().getStreamY();
    driverY = controller.getLeftThumbstick().getStreamX();
    BreakerInputStream translationalMag =
        BreakerInputStream.hypot(driverX, driverY)
            .clamp(1.0)
            .deadband(Constants.DriverConstants.TRANSLATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));

    BreakerInputStream translationalTheta = BreakerInputStream.atan(driverX, driverY);

    driverX = translationalMag.scale(translationalTheta.map(Math::cos));
    driverY = translationalMag.scale(translationalTheta.map(Math::sin));

    driverOmega = controller.getRightThumbstick().getStreamX()
            .clamp(1.0)
            .deadband(Constants.DriverConstants.ROTATIONAL_DEADBAND, 1.0)
            .map(new BreakerLinearizedConstrainedExponential(0.0, 3.0, true))
            .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));

    teleopSwerveRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> teleopSwerveRequest.withVelocityX(driverX.get()).withVelocityY(driverY.get()).withRotationalRate(driverOmega.get())));
  
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
