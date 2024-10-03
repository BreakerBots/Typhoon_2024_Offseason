package frc.robot.BreakerLib.auto;

import choreo.Choreo.ControlFunction;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class BreakerSwerveAutoController implements ControlFunction<SwerveSample> {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public BreakerSwerveAutoController(
        PIDController xController,
        PIDController yController,
        PIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.thetaController = thetaController;
    }

    @Override
    public ChassisSpeeds apply(Pose2d t, SwerveSample u) {
        double xFF = u.vx;
        double yFF = u.vy;
        double rotationFF = u.omega;

        double xFeedback = xController.calculate(t.getX(), u.x);
        double yFeedback = yController.calculate(t.getY(), u.y);
        double rotationFeedback = thetaController.calculate(t.getRotation().getRadians(),
           u.heading);

        ChassisSpeeds out = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            t.getRotation()
        );

        return out;
    }
}
