package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.drive.PIDToPosesCommand;
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
import frc.robot.subsystems.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.apriltag.Vision;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
@Logged
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
    private final ObjectPoseEstimator objectPoseEstimator;

    public Superstructure(Drive drive, Vision vision, Shooter shooter, Hood hood, IntakeDeploy intakeDeploy, IntakeRollers intakeRollers, Kicker kicker, Conveyor conveyor, Climber climber, ObjectPoseEstimator objectPoseEstimator) {
        this.drive = drive;
        this.vision = vision;
        this.shooter = shooter;
        this.hood = hood;
        this.intakeDeploy = intakeDeploy;
        this.intakeRollers = intakeRollers;
        this.kicker = kicker;
        this.conveyor = conveyor;
        this.climber = climber;
        this.objectPoseEstimator = objectPoseEstimator;
    }

    private boolean isPathFollowing = false;
    private boolean superstructureDone = false;
    private boolean driveReady = false;
    private boolean kitbotMode = false;
    private boolean intakeDeployed = false;
    public boolean shootOnTheMove = false;

    public double maintainHeadingEpsilon = 0.25;

    private State state = State.TUCK;

    public Setpoint hoodSetpoint = Hood.ZERO;
    public Setpoint shooterSetpoint = Shooter.STOP;
    public Rotation2d headingSetpoint = new Rotation2d();

    @Override
    public void periodic() {
        updateShooterSetpoint();
        updateHoodSetpoint();
        updateHeadingSetpoint();
        SmartDashboard.putBoolean("Near Trench", FieldLayout.nearTrench(drive.getPose(), drive.getFieldRelativeChassisSpeeds()));
    }

    public void updateShooterSetpoint() {
      if (visionValid() == true) {
        shooterSetpoint = 
            Setpoint.withVelocitySetpoint(
              Units.RotationsPerSecond.of(
              ShotCalculator.getInstance(drive)
              .getParameters()
              .flywheelSpeed()));
      }
      else if (visionValid() == false) {
        shooterSetpoint = Shooter.KITBOT;
        kitbotMode = true;
        ControlBoard.getInstance(drive, this).setRumble(true);
      }
    }

    public void updateHoodSetpoint() {
      if (visionValid() && !FieldLayout.nearTrench(drive.getPose(), drive.getFieldRelativeChassisSpeeds())) {
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

    public void updateHeadingSetpoint() {
      boolean passing = !FieldLayout.distanceFromAllianceWall(Units.Meters.of(drive.getPose().getX()), RobotConstants.isRedAlliance).lte(FieldLayout.kAllianceZoneX.minus(Units.Inches.of(14)));
      if (!passing) {
      headingSetpoint = shootOnTheMove ? ShotCalculator.getInstance(drive).getParameters().heading() : ShotCalculator.getStationaryAimedPose(drive.getPose().getTranslation()).getRotation();
      }
      else {
        headingSetpoint = ShotCalculator.getInstance(drive).getParameters().heading();
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
            .finallyDo(() -> {
                intakeRollers.setpointCommand(IntakeRollers.IDLE);
                conveyor.setpointCommand(Conveyor.IDLE);
                kicker.setpointCommand(Kicker.IDLE);
            })
            .withName("Spit");
    }

    public Command waitUntilSafeToShoot() {
      return Commands.waitUntil(() -> shooter.spunUp() 
      && hood.nearPositionSetpoint() 
      && (kitbotMode || RobotState.isAutonomous() || drive.getRotation().getMeasure().isNear(headingSetpoint.getMeasure(), Units.Degrees.of(5.0))));
    }

    public Command shoot() {
      return Commands.parallel(
        conveyor.setpointCommand(Conveyor.FEED_FORWARD),
        kicker.setpointCommand(Kicker.FEED_FORWARD)).
        withName("Shoot");
    }

    public Command shootWhenReady() {
      return Commands.sequence(
                Commands.runOnce(() -> { 
                  if (state != State.SHOOTING) waitUntilSafeToShoot();}),
                shoot(),
                setState(State.SHOOTING),
                Commands.waitUntil(() -> false))
                .finallyDo(() -> {
                  conveyor.setpointCommand(Conveyor.IDLE); 
                  kicker.setpointCommand(Kicker.IDLE);
                });
    }

    public Command deployIntake() {
      return Commands.sequence(setIntakeStatus(true), intakeDeploy.setpointCommand(IntakeDeploy.DEPLOY))
      .withName("Intake Deploy");
    }

    public Command runIntakeIfDeployed() {
      return Commands.sequence(Commands.either(
          intakeRollers.setpointCommand(IntakeRollers.INTAKE),
          Commands.sequence(
              deployIntake(),
              intakeRollers.setpointCommand(IntakeRollers.INTAKE)),
          () -> intakeDeployed),
          setState(State.INTAKING),
          Commands.waitUntil(() -> false))
          .withName("Intaking").finallyDo(() -> intakeRollers.setpointCommand(IntakeRollers.IDLE).withName("End Intake"));
    }

    public Command tuck() {
      return Commands.sequence(
          intakeDeploy.setpointCommand(IntakeDeploy.STOW_CLEAR),
          setIntakeStatus(false),
          setState(State.TUCK))
        .withName("Tuck");
    }

    public Command driveBrake() {
      return Commands.sequence(
        Commands.waitUntil(() -> drive.getRotation().getMeasure().isNear(headingSetpoint.getMeasure(), Units.Degrees.of(5.0))),
        drive.brake()
      );
    }

    public Command climb() {
      return Commands.defer(
          () -> {
            Pose2d ladderSide =
                FieldLayout.handleAllianceFlip(drive.getPose().nearest(List.of(FieldLayout.handleAllianceFlip(FieldLayout.leftTower, RobotConstants.isRedAlliance), FieldLayout.handleAllianceFlip(FieldLayout.rightTower, RobotConstants.isRedAlliance))), RobotConstants.isRedAlliance);

            boolean isLeftTower = ladderSide.equals(FieldLayout.leftTower);

            Pose2d initialPose = FieldLayout.handleAllianceFlip(
                ladderSide.transformBy(
                    new Transform2d(
                        SuperstructureConstants.climberOffset
                            .toTranslation2d()
                            .plus(new Translation2d(
                                Units.Inches.of(!isLeftTower ? -5.0 : 5.0),
                                Units.Inches.of(0.0))),
                                Rotation2d.kZero)), 
                        RobotConstants.isRedAlliance);

            Pose2d finalPose = initialPose 
                  .transformBy(
                    new Transform2d(
                        Units.Inches.of(!isLeftTower ? 5.0 : -5.0),
                        Units.Inches.of(!isLeftTower ? 5.875 / 2.0 : -5.875 / 2.0),
                        !isLeftTower ? Rotation2d.kZero : Rotation2d.k180deg));
            if (isLeftTower) finalPose = finalPose.transformBy(new Transform2d(SuperstructureConstants.climberOffset.getMeasureX().times(2.0), Units.Inches.of(0.0), Rotation2d.kZero));
              
            if (isLeftTower) {
              return Commands.sequence(
                  setState(State.CLIMBING),
                  climber.setpointCommand(Climber.CLIMB).withName("Climber Raise"),
                  new PIDToPoseCommand(drive, this, finalPose).withName("Final Align"),
                  climber.setpointCommand(Climber.CLIMB).withName("Climbing")
              );
            } else {
              return Commands.sequence(
                  setState(State.CLIMBING),
                  new PIDToPoseCommand(drive, this, initialPose).withName("Initial Align"),
                  climber.setpointCommand(Climber.CLIMB).withName("Climber Raise"),
                  new PIDToPoseCommand(drive, this, finalPose).withName("Final Align"),
                  climber.setpointCommand(Climber.ZERO).withName("Climbing")
              );
            }
          },
          Set.of(drive, climber)
      ).withName("Climb Sequence");
    }

    public Command collectFuelCommand() {
        return Commands.defer(() -> {

            var clusters = objectPoseEstimator.getOrderedClusters();
            if (clusters.isEmpty() || IntakeRollerConstants.numberOfFuel >= IntakeRollerConstants.fuelLimit) {
                return Commands.none();
            }

            List<Pose2d> poses = new ArrayList<>();
            Translation2d currentTrans = drive.getPose().getTranslation();

            for (Translation2d t : clusters) {
                Rotation2d r = t.minus(currentTrans).getAngle();
                poses.add(new Pose2d(t, r));
                currentTrans = t;
            }

            return new PIDToPosesCommand(drive, this, poses)
                .andThen(collectFuelCommand());

        }, Set.of(drive));
    }



    public static enum State {
      TUCK,
      SHOOTING,
      INTAKING,
      SPIT,
      DEPLOYED,
      CLIMBING,
      SHOOTINTAKE,
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

    public Command setIntakeStatus(boolean status) {
      return Commands.runOnce(() -> intakeDeployed = status);
    }

    public Command toggleSOTM() {
      return Commands.runOnce(() -> shootOnTheMove = !shootOnTheMove);
    }
  
    public State getState() {
      return state;
    }

    public boolean shouldHeadingLock() {
      return (state != State.INTAKING);
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
