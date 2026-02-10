package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.Conveyor.Conveyor;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class Superstructure extends SubsystemBase {
    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Hood hood;
    private final IntakeDeploy intakeDeploy;
    private final IntakeRollers intakeRollers;
    private final Kicker kicker;
    private final Conveyor conveyor;

    public Superstructure(Drive drive, Vision vision, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, Kicker kicker, Conveyor conveyor) {
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.hood = hood;
        this.intakeDeploy = intakeDeploy;
        this.intakeRollers = intakeRollers;
        this.kicker = kicker;
        this.conveyor = conveyor;
    }

    private boolean isPathFollowing = false;
    private boolean superstructureDone = false;
    private boolean driveReady = false;
    private boolean kitbotMode = false;

    private State state = State.TUCK;

    public Setpoint hoodSetpoint = Hood.ZERO;
    public Setpoint shooterSetpoint = Shooter.STOP;

    @Override
    public void periodic() {
        updateShooterSetpoint();
        updateHoodSetpoint();
    }

    public void updateShooterSetpoint() {
      if (visionValid()) {
        shooterSetpoint = 
            Setpoint.withVelocitySetpoint(
              Units.RotationsPerSecond.of(
              ShotCalculator.getInstance(drive)
              .getParameters()
              .flywheelSpeed()));
      }
      else {
        shooterSetpoint = Shooter.KITBOT;
        ControlBoard.getInstance(drive, this).setRumble(true);
      }
    }

    public void updateHoodSetpoint() {
      Pose2d lookaheadPose = drive.getLookaheadPose(SuperstructureConstants.lookaheadTrenchTime);
      if (visionValid() && !FieldLayout.nearTrench(lookaheadPose, RobotConstants.isRedAlliance)) {
        hoodSetpoint = 
            Setpoint.withMotionMagicSetpoint(
              Units.Degrees.of(
              ShotCalculator.getInstance(drive)
              .getParameters()
              .flywheelSpeed()));
      }
      else {
        hoodSetpoint = Hood.KITBOT;
      }
    }

    public static enum State {
      TUCK,
      SHOOTING
    }

    public boolean visionValid() {
      double time = vision.timeSinceLastTargetSeen();
      ChassisSpeeds speeds = drive.getRobotRelativeChassisSpeeds();
      double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      
      if (totalSpeed < 1.0) {
        return time < 1.0;
      }
      else {
        return time < 0.5;
      }
    }
  
    public Command setState(State state) {
      return Commands.runOnce(() -> this.state = state);
    }
  
    public State getState() {
      return state;
    }

    public void setPathFollowing(boolean isFollowing) {
		  isPathFollowing = isFollowing;
	  }

    public void setSuperstructureDone(boolean valToSet) {
		  superstructureDone = valToSet;
	  }

    public void setDriveReady(boolean valToSet) {
		  driveReady = valToSet;
	  }

    public boolean getSuperstructureDone() {
      return superstructureDone;
    }

}
