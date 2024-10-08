// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.BreakerLib.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.loging.BreakerLog;
import frc.robot.subsystems.Intake2.IntakeRollerState;
import frc.robot.subsystems.Intake2.IntakeState.IntakeSetpoint;
// import frc.robot.subsystems.vision.NoteVision;
import frc.robot.subsystems.vision.ZED;
import frc.robot.subsystems.vision.ZED.TrackedObject;

import static frc.robot.Constants.IntakeConstants.MAX_SMART_ROLLER_ENABLE_NOTE_DISTANCE;
import static frc.robot.Constants.IntakeConstants.SMART_ROLLER_MAX_WAIT_FOR_DET;
import static frc.robot.Constants.IntakeConstants2.*;
import static java.lang.Math.round;

import java.util.Currency;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake2 extends SubsystemBase {
  /** Creates a new Intake2. */
  private IntakeSetpoint setpoint;
  private BreakerBeamBreak beamBreak;
  private MotionMagicVoltage motionMagicRequest;
  private Follower pivotFollowerRequest;
  private TalonFX leftPivot, rightPivot, rollerMotor;
  private Supplier<Double> pivotPosSup;
  public Intake2() {
  }


  public IntakeState getCurrentState() {
    return new IntakeState(Rotation2d.fromRotations(pivotPosSup.get()), setpoint.goalState.rollerState);
  }

  public boolean atSetpoint() {
    return setpoint.isSatisfied(getCurrentState());
  }

  public IntakeSetpoint getSetpoint() {
    return setpoint;
  }

  public Command setStateCommand(IntakeSetpoint setpoint, boolean waitForSuccess) {
    return Commands.runOnce(() -> {setState(setpoint);}, this).andThen(Commands.waitUntil(() -> atSetpoint() || !waitForSuccess));
  }
  
  private void setState(IntakeSetpoint setpoint) {
    this.setpoint = setpoint;
    rollerMotor.set(setpoint.goalState.rollerState.getMotorDutyCycle());
    leftPivot.setControl(motionMagicRequest.withPosition(setpoint.goalState.pivotAngle.getRotations()));
    rightPivot.setControl(pivotFollowerRequest);
  }

  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public static record IntakeState(Rotation2d pivotAngle, IntakeRollerState rollerState) {

    public IntakeSetpoint toSetpoint(Measure<Angle> pivotTolerence) {
      return new IntakeSetpoint(this, pivotTolerence);
    }

    public IntakeSetpoint toSetpoint() {
      return new IntakeSetpoint(this, PIVOT_DEFAULT_TOLERENCE);
    }

    public static record IntakeSetpoint(IntakeState goalState, Measure<Angle> pivotTolerence) {
      public static final IntakeSetpoint EXTENDED_INTAKEING = new IntakeState(PIVOT_EXTENDED_ANGLE, IntakeRollerState.INTAKEING).toSetpoint(PIVOT_EXTENDED_TOLERENCE); 
      public static final IntakeSetpoint EXTENDED_EXTAKEING = new IntakeState(PIVOT_EXTENDED_ANGLE, IntakeRollerState.EXTAKEING).toSetpoint(PIVOT_EXTENDED_TOLERENCE);
      public static final IntakeSetpoint EXTENDED_NEUTRAL = new IntakeState(PIVOT_EXTENDED_ANGLE, IntakeRollerState.NEUTRAL).toSetpoint(PIVOT_EXTENDED_TOLERENCE); 
      public static final IntakeSetpoint RETRACTED_INTAKEING = new IntakeState(PIVOT_RETRACTED_ANGLE, IntakeRollerState.INTAKEING).toSetpoint(PIVOT_RETRACTED_TOLERENCE); 
      public static final IntakeSetpoint RETRACTED_EXTAKEING = new IntakeState(PIVOT_RETRACTED_ANGLE, IntakeRollerState.EXTAKEING).toSetpoint(PIVOT_RETRACTED_TOLERENCE);
      public static final IntakeSetpoint RETRACTED_NEUTRAL = new IntakeState(PIVOT_RETRACTED_ANGLE, IntakeRollerState.NEUTRAL).toSetpoint(PIVOT_RETRACTED_TOLERENCE);
      public static final IntakeSetpoint AMP_INTAKEING = new IntakeState(PIVOT_AMP_ANGLE, IntakeRollerState.INTAKEING).toSetpoint(PIVOT_AMP_TOLERENCE); 
      public static final IntakeSetpoint AMP_EXTAKEING = new IntakeState(PIVOT_AMP_ANGLE, IntakeRollerState.EXTAKEING).toSetpoint(PIVOT_AMP_TOLERENCE);
      public static final IntakeSetpoint AMP_NEUTRAL = new IntakeState(PIVOT_AMP_ANGLE, IntakeRollerState.NEUTRAL).toSetpoint(PIVOT_AMP_TOLERENCE);  


      public boolean isSatisfied(IntakeState currentState) {
        return MathUtil.isNear(goalState.pivotAngle.getRotations(), currentState.pivotAngle.getRotations(), pivotTolerence.in(Units.Rotations), -0.5, 0.5) && goalState.rollerState == currentState.rollerState;
      }
    }
  }

  private void applyRollerState(IntakeRollerState rollerState) {
    rollerMotor.set(rollerState.getMotorDutyCycle());
  }

  public static enum IntakeRollerState {
    INTAKEING(-0.8),
    EXTAKEING(1.0),
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakeRollerState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }

  @Override
  public void periodic() {
    IntakeState currentState = getCurrentState();
    if (RobotState.isDisabled()) {
      double curPos = currentState.pivotAngle.getRadians();
      double errToExt = Math.abs(PIVOT_EXTENDED_ANGLE.getRadians() - curPos);
      double errToRet = Math.abs(PIVOT_RETRACTED_ANGLE.getRadians() - curPos);
      if (errToExt < errToRet) {
        setState(IntakeSetpoint.EXTENDED_NEUTRAL);
      } else {
        setState(IntakeSetpoint.RETRACTED_NEUTRAL);
      }
    }
    BreakerLog.log("Intake/HasNote", beamBreak.isBroken());
    BreakerLog.log("Intake/Position", currentState.pivotAngle.getRotations());
    BreakerLog.log("Intake/Setpoint/Position", setpoint.goalState.pivotAngle);
    BreakerLog.log("Intake/Setpoint/RollerState", setpoint.goalState.rollerState);
    BreakerLog.log("Intake/Setpoint/Tolerence", setpoint.pivotTolerence.in(Units.Rotation));
    BreakerLog.log("Intake/Setpoint/AtSetpoint", atSetpoint());
    BreakerLog.log("Intake/RollerMotor", rollerMotor);
    BreakerLog.log("Intake/LeftPivotMotor", leftPivot);
    BreakerLog.log("Intake/RightPivotMotor", rightPivot);
  }

 

  /** Dynaicly enables and disables intake rollers depeding on note visablity */
  public Command smartRollerControlCommand(ZED zed, IntakeSetpoint endSetpoint, BooleanSupplier endCondition) {
    Timer timer = new Timer();
    return new FunctionalCommand(()->{
      timer.stop();
      timer.reset();
    }, 
      () -> {
        boolean isApplicableToCurrentState = RobotState.isTeleop();
        if (isApplicableToCurrentState) {
          boolean shouldRun = false;//noteVision.hasTarget();
          if (!shouldRun) {
            for (TrackedObject obj : zed.getTrackedObjects()) {
              double dist = obj.position().getRobotToObject(true).getNorm();
              if (dist <= MAX_SMART_ROLLER_ENABLE_NOTE_DISTANCE.in(Units.Meters)) {
                shouldRun = true;
                break;
              }
            }
          }
          if (!shouldRun) {
            timer.start();
          } else {
            timer.stop();
            timer.reset();
          }

          if ((shouldRun || !timer.hasElapsed(SMART_ROLLER_MAX_WAIT_FOR_DET)) && setpoint.goalState.rollerState != IntakeRollerState.INTAKEING) {
            setState(IntakeSetpoint.EXTENDED_INTAKEING);
          } else if (setpoint.goalState.rollerState != IntakeRollerState.NEUTRAL) {
            setState(IntakeSetpoint.EXTENDED_NEUTRAL);
          }
        } else {
          timer.stop();
          timer.reset();
          setState(IntakeSetpoint.EXTENDED_INTAKEING);
        }
      }, (Boolean cancled) -> setState(endSetpoint), endCondition, this);
  }
}
