// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static java.lang.Math.log;

import java.util.function.Function;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import static frc.robot.Constants.ShooterConstants.*;

/** Add your docs here. */
public class ShooterTarget2 {
    private Translation3d blueTargetPoint;
    private final Drivetrain drivetrain;
    public static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    private Function<Double, Double> angleFunction;
    private Function<Double, Double> speedFunction;
    public ShooterTarget2() {

    }

    public Command SmartSpool(Drivetrain drive, Intake intake, Hopper hopper) {
        
    }

    public Translation3d getTargetPosition() {
        return null;
    }

    private FireingSolution getMoveingFireingSolution(Translation2d robotTranslation, ChassisSpeeds robotSpeeds) {
        FireingSolution stationarySolution = getStationaryFireingSolution(robotTranslation);
        BreakerVector3 stationaryLaunchNoteVelVec = stationarySolution.toNoteVelocity();
        BreakerVector3 corectedVec = stationaryLaunchNoteVelVec.minus(new BreakerVector3(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, 0.0));
        return FireingSolution.fromNoteVelocity(corectedVec);
    }

    private FireingSolution getStationaryFireingSolution(Translation2d robotTranslation) {
        Translation2d targetPos = getTargetPosition().toTranslation2d();
        double distance = targetPos.getDistance(robotTranslation);
        Rotation2d angle = robotTranslation.minus(targetPos).getAngle();
        return new FireingSolution(angle, getStationaryShooterState(distance));
    }


    private ShooterState getStationaryShooterState(double distanceMeters) {
        return new ShooterState(Rotation2d.fromRadians(angleFunction.apply(distanceMeters)), Units.RadiansPerSecond.of(speedFunction.apply(distanceMeters)));
    }

    public record FireingSolution(Rotation2d robotYaw, ShooterState shooterState) {
        public static FireingSolution fromNoteVelocity(BreakerVector3 noteVelocityVector) {
            Rotation3d vecRot = noteVelocityVector.getVectorRotation();
            double yaw = vecRot.getZ();
            double rawPitch = vecRot.getY();
            Rotation2d pitch = Rotation2d.fromRadians(rawPitch).plus(Rotation2d.fromDegrees(180));
            Measure<Velocity<Angle>> flywheelAngVel = Units.RadiansPerSecond.of((noteVelocityVector.getMagnitude() / FLYWHEEL_RADIUS.in(Units.Meters)) * NOTE_LAUCH_VEL_LOSS_SCAILAR);
            return new FireingSolution(Rotation2d.fromRadians(yaw), new ShooterState(pitch, flywheelAngVel));
        }
        public BreakerVector3 toNoteVelocity() {
            double velMPS = (shooterState.flywheelVel().in(Units.RadiansPerSecond) * FLYWHEEL_RADIUS.in(Units.Meters)) / NOTE_LAUCH_VEL_LOSS_SCAILAR;
            return new BreakerVector3(velMPS, new Rotation3d(0.0, shooterState.pitchAngle().minus(Rotation2d.fromDegrees(180)).getRadians(), robotYaw.getRadians()));
        }
    }
}
