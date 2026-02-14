package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.apriltag.Vision;

public class Superstructure extends SubsystemBase {
    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Hood hood;
    private final IntakeDeploy intakeDeploy;
    private final IntakeRollers intakeRollers;
    private final Kicker kicker;
    private final Conveyor conveyor;
    private final Climber climber;

    public Superstructure(Drive drive, Vision vision, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, Kicker kicker, Conveyor conveyor, Climber climber) {
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.hood = hood;
        this.intakeDeploy = intakeDeploy;
        this.intakeRollers = intakeRollers;
        this.kicker = kicker;
        this.conveyor = conveyor;
        this.climber = climber;
    }

    private boolean isPathFollowing = false;
    private boolean superstructureDone = false;
    private boolean driveReady = false;
    private boolean kitbotMode = false;
    private boolean intakeDeployed = false;

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
        setState(State.KITBOT);
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


    /**
     * stops intake rollers, conveyor, and kicker
     */
    public Command idleRollers() {
      return Commands.parallel(
                      intakeRollers.setpointCommand(IntakeRollers.IDLE),
                      conveyor.setpointCommand(Conveyor.IDLE),
                      kicker.setpointCommand(Kicker.IDLE)
              .withName("Idle Rollers")
      );
    }

    public Command zero() {
      return Commands.runOnce(() -> {
            intakeDeploy.setCurrentPosition(IntakeDeployConstants.kStowClearPosition);
            hood.setCurrentPosition(HoodConstants.kMinAngle);
            climber.setCurrentPosition(ClimberConstants.kZeroHeight);
          })
      .withName("Zero");
    }

    public Command spit() {
      return Commands.parallel(
            intakeDeploy.setpointCommand(IntakeDeploy.EXHAUST),
            intakeRollers.setpointCommand(IntakeRollers.EXHAUST),
            conveyor.setpointCommand(Conveyor.FEED_BACKWARDS),
            kicker.setpointCommand(Kicker.FEED_BACKWARDS),
            setState(State.SPIT),
            Commands.waitUntil(() -> false))
            .handleInterrupt(() -> {
                intakeRollers.setpointCommand(IntakeRollers.IDLE);
                conveyor.setpointCommand(Conveyor.IDLE);
                kicker.setpointCommand(Kicker.IDLE);
            })
            .withName("Spit");
    }

    public Command waitUntilSafeToShoot() {
      return Commands.waitUntil(() -> shooter.spunUp() 
      && hood.nearPositionSetpoint() 
      && drive.getRotation().getMeasure().isNear(ShotCalculator.getInstance(drive).getParameters().heading().getMeasure(), Units.Degrees.of(5.0)));
    }

    public Command shoot() {
      return Commands.parallel(
        conveyor.setpointCommand(Conveyor.FEED_FORWARD),
        kicker.setpointCommand(Kicker.FEED_FORWARD).
        withName("Shoot"));
    }

    public Command shootWhenReady() {
      return Commands.parallel(turnToHub(), Commands.sequence(
                Commands.runOnce(() -> { 
                  if (state != State.SHOOTING) waitUntilSafeToShoot();}),
                Commands.run(() -> shoot()),
                setState(State.SHOOTING)));
    }

    public Command deployIntake() {
      intakeDeployed = true;
      return intakeDeploy.setpointCommand(IntakeDeploy.DEPLOY)
      .withName("Intake Deploy");
    }

    public Command runIntakeIfDeployed() {
      return Commands.either(
          intakeRollers.setpointCommand(IntakeRollers.INTAKE),
          Commands.sequence(
              deployIntake(),
              intakeRollers.setpointCommand(IntakeRollers.INTAKE)),
          () -> intakeDeployed)
          .withName("Intaking");
    }

    public Command tuck() {
      intakeDeployed = false;
      return Commands.runOnce(() -> {
          intakeDeploy.setpointCommand(IntakeDeploy.STOW_CLEAR);
          setState(State.TUCK);
        })
        .withName("Tuck");
    }

    public Command turnToHub() {
      if (state != State.KITBOT)
        return new PIDToPoseCommand(drive, this, new Pose2d(drive.getPose().getX(), drive.getPose().getY(), ShotCalculator.getInstance(drive).getParameters().heading()));
      return Commands.none();
    }

    public static enum State {
      TUCK,
      SHOOTING,
      INTAKING,
      KITBOT,
      SPIT
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
