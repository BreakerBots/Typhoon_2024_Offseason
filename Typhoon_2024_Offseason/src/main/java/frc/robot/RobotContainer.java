// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.InputStream;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.commands.amp.ScoreAmp;
import frc.robot.commands.intake.HopperToIntakeHandoff;
import frc.robot.commands.intake.IntakeAndHold;
import frc.robot.commands.intake.IntakeAssist;
import frc.robot.commands.intake.IntakeForShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePivotState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterTarget;
import frc.robot.subsystems.vision.ZED;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {
  public static final BreakerXboxController controller = new BreakerXboxController(0);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter(drivetrain::getChassisAccels);
  private final Hopper hopper = new Hopper();
  
  private final ZED zed = new ZED(null, null);

  private BreakerInputStream driverX, driverY, driverOmega;
  private SwerveRequest.FieldCentric teleopSwerveRequest;
  private final ShooterTarget SPEAKER = new ShooterTarget(drivetrain, shooter, hopper);
  private final ShooterTarget PASS = new ShooterTarget(drivetrain, shooter, hopper);
  public RobotContainer() {
    shooter.setDefaultCommand(SPEAKER.runSmartSpool(intake));
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

    BooleanSupplier hasNoNote = () -> !intake.hasNote() && !hopper.hasNote();
    BooleanSupplier intakeOnlyHasNote = () -> intake.hasNote() && !hopper.hasNote();
    BooleanSupplier hopperOnlyHasNote = () -> !intake.hasNote() && hopper.hasNote();
    BooleanSupplier errorState = () -> intake.hasNote() && hopper.hasNote();
    Trigger globalOverride = controller.getStartButton();

    // AMP CONTROLS
    controller.getLeftBumper()//intake amp
      .and(hasNoNote)
      .onTrue(new IntakeAndHold(intake, true)
      .deadlineWith(new ConditionalCommand(
        Commands.print("Intake Assist Overridden"), 
        new IntakeAssist(drivetrain, intake, hopper, zed, driverX, driverY, driverOmega), 
        globalOverride
        )));
    controller.getLeftBumper()//handoff from hopper to intake
      .and(hopperOnlyHasNote)
      .onTrue(new HopperToIntakeHandoff(hopper, intake, true));
    controller.getLeftBumper()//score amp
      .and(intakeOnlyHasNote)
      .and(() -> intake.getState().getPivotState() == IntakePivotState.RETRACTED)
      .onTrue(new ScoreAmp(null, null));
    controller.getLeftBumper()//prep for amp score
      .and(intakeOnlyHasNote)
      .and(() -> intake.getState().getPivotState() != IntakePivotState.RETRACTED)
      .onTrue(intake.setStateCommand(IntakeState.RETRACTED_NEUTRAL, false));

    // SHOOTER CONTROLS
    controller.getRightBumper()//intake shooter
      .and(hasNoNote)
      .onTrue(new IntakeForShooter(intake, hopper, shooter)
      .deadlineWith(new ConditionalCommand(
        Commands.print("Intake Assist Overridden"), 
        new IntakeAssist(drivetrain, intake, hopper, zed, driverX, driverY, driverOmega), 
        globalOverride)));
    controller.getRightBumper()//handoff from intake to hopper
      .and(intakeOnlyHasNote)
      .onTrue(new IntakeForShooter(intake, hopper, shooter));
    controller.getRightBumper()
      .and(hopperOnlyHasNote)
      .onTrue(SPEAKER.shootWhileMoveing(driverX, driverY));

  
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
