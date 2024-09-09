package frc.robot.BreakerLib.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.choreo.ChoreoControlFunction;
import frc.robot.choreo.trajectory.ChoreoTrajectoryState;

public class BreakerSwerveAutoController implements ChoreoControlFunction {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public BreakerSwerveAutoController(PIDController xController, PIDController yController, PIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.thetaController = thetaController;
    }

    @Override
    public ChassisSpeeds apply(Pose2d pose, ChoreoTrajectoryState referenceState) {
        double xFF = referenceState.velocityX;
        double yFF = referenceState.velocityY;
        double rotationFF = referenceState.angularVelocity;

        double xFeedback = xController.calculate(pose.getX(), referenceState.x);
        double yFeedback = yController.calculate(pose.getY(), referenceState.y);
        double rotationFeedback = thetaController.calculate(pose.getRotation().getRadians(),
            referenceState.heading);

        ChassisSpeeds out = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            pose.getRotation()
        );

        return out;
    }
}
