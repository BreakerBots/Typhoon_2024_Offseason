// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class AutoHelper {


    public static Command AmpSide3Note(Drivetrain drivetrain) {
        AutoFactory factory = drivetrain.getAutoFactory();
        
        AutoTrajectory startToFirstNote = null;


        startToFirstNote.done().onTrue()
        return null;
    }
}
