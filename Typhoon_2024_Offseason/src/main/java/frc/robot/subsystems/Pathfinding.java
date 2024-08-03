// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.vision.ZED;

/** Add your docs here. */
public class Pathfinding {
    private Drivetrain drivetrain;
    private ZED zed;
    private Supplier<Rotation2d> scanRotOverrideSup;
    
    public void arbitraryPathToPose(Pose2d targetPose, PathConstraints constraints) {
        AutoBuilder.pathfindToPose(targetPose, constraints).alongWith(Commands.runOnce(() -> {PPHolonomicDriveController.setRotationTargetOverride()}, null))
    }

    public Command obsticleAvoidanceScanRotationOverride(double endOverrideDistanceFromGoal) {
        Commands.runOnce(() -> PPHolonomicDriveController.setRotationTargetOverride(scanRotOverrideSup)).andThen(new WaitCommand());
        
    }

}
