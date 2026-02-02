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
import frc.robot.shooting.ShotCalculator;
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

    public Superstructure(Drive drive, Vision vision, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers) {
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.hood = hood;
        this.intakeDeploy = intakeDeploy;
        this.intakeRollers = intakeRollers;
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
      if (vision.timeSinceLastTargetSeen() < .5) {
        shooterSetpoint = 
            Setpoint.withVelocitySetpoint(
            Units.RotationsPerSecond.of(
            ShotCalculator.getInstance(drive)
            .getParameters()
            .flywheelSpeed()));
      }
    }

    public void updateHoodSetpoint() {
      if (vision.timeSinceLastTargetSeen() < .5) {
        hoodSetpoint = 
            Setpoint.withMotionMagicSetpoint(
              Units.Degrees.of(
              ShotCalculator.getInstance(drive)
              .getParameters()
              .flywheelSpeed()));
      }
      ChassisSpeeds speeds = drive.getRobotRelativeChassisSpeeds();
      Transform2d speedsPose = new Transform2d(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond,
						Rotation2d.fromRadians(speeds.omegaRadiansPerSecond))
				.times(SuperstructureConstants.lookaheadTrenchTime.in(Units.Seconds));
      Pose2d lookaheadPose = drive.getPose().transformBy(speedsPose);
      if (FieldLayout.nearTrench(lookaheadPose, RobotConstants.isRedAlliance)) {
        hoodSetpoint = Hood.KITBOT;
      }
    }

    public static enum State {
      TUCK,
      SHOOTING
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
