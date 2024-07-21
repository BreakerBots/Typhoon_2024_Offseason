// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/** Add your docs here. */
public class BreakerUnits {
    public static final Unit<Velocity<Velocity<Angle>>> RotationsPerSecondPerSecond = Units.RotationsPerSecond.per(Units.Second);
}
