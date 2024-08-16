// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain;
import frc.robot.subsystems.vision.gtsam.GtsamInterface;

import static frc.robot.Constants.DriveConstants.*;

public class Drivetrain extends BreakerSwerveDrivetrain {
  /** Creates a new Drive. */

  private ChassisSpeeds prevChassisSpeeds = new ChassisSpeeds();
  private ChassisAccels chassisAccels = new ChassisAccels();
  private SwerveDriveWheelPositions prevModulePositions;
  private GtsamInterface gtsam;

  public Drivetrain(GtsamInterface gtsam) {
    super(DRIVETRAIN_CONSTANTS, FRONT_LEFT_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_CONSTANTS, BACK_LEFT_MODULE_CONSTANTS, BACK_RIGHT_MODULE_CONSTANTS);
    this.gtsam = gtsam;
    registerTelemetry(this::odometryThreadCallback);
  }

  private void odometryThreadCallback(SwerveDriveState state) {
  
    chassisAccels = ChassisAccels.fromDeltaSpeeds(prevChassisSpeeds, state.speeds, state.OdometryPeriod);
    prevChassisSpeeds = state.speeds;

    if (prevModulePositions == null) {
      var posArr = new SwerveModulePosition[state.ModuleStates.length];
      for (int i = 0; i < posArr.length; i++) {
        posArr[i] = new SwerveModulePosition();
      }
      prevModulePositions = new SwerveDriveWheelPositions(posArr);
    }
    SwerveDriveWheelPositions now = new SwerveDriveWheelPositions(m_modulePositions);
    Twist2d twist = m_kinematics.toTwist2d(prevModulePositions, now);
    prevModulePositions = now;
    var twist3 = new Twist3d(
            twist.dx, twist.dy, 0,
            0, 0, twist.dtheta
        );
    gtsam.sendOdomUpdate(WPIUtilJNI.now(), twist3, new Pose3d(state.Pose));
    
  }

  public ChassisAccels getChassisAccels() {
    try {
      m_stateLock.writeLock().lock();
      return chassisAccels;
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }
  
}
