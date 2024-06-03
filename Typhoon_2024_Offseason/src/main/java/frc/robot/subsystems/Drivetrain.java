// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import static frc.robot.Constants.DriveConstants.*;

public class Drivetrain extends BreakerSwerveDrivetrain {
  /** Creates a new Drive. */

  public Drivetrain() {
    super(DRIVETRAIN_CONSTANTS, FRONT_LEFT_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_CONSTANTS, BACK_LEFT_MODULE_CONSTANTS, BACK_RIGHT_MODULE_CONSTANTS);
  }

}
