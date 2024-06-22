// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class Notes {

    /* Numbers start from amp as 0 */
    public static enum AutonNotePosition {
        WING_1(new Translation2d(0.0, 0.0)),
        WING_2(new Translation2d(0.0, 0.0)),
        WING_3(new Translation2d(0.0, 0.0)),
        CENTER_1(new Translation2d(0.0, 0.0)),
        CENTER_2(new Translation2d(0.0, 0.0)),
        CENTER_3(new Translation2d(0.0, 0.0)),
        CENTER_4(new Translation2d(0.0, 0.0)),
        CENTER_5(new Translation2d(0.0, 0.0));
        private Translation2d blueAlliancePos;
        private AutonNotePosition(Translation2d blueAlliancePos) {
            this.blueAlliancePos = blueAlliancePos;
        }

        public Translation2d getPosition() {
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent() && ally.get() == Alliance.Red) {
                return GeometryUtil.flipFieldPosition(blueAlliancePos);
            }
            return blueAlliancePos;
        }
    }
}
