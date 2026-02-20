package frc.robot.controlboard;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.FieldLayout;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.State;

import java.util.function.Supplier;

public class ControlBoard {
    private Drive drive;
    private Superstructure superstructure;
    public ControlBoard(Drive drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;
    }

    private static ControlBoard instance = null;

    public static ControlBoard getInstance(Drive drive, Superstructure superstructure) {
        if (instance == null) {
            instance = new ControlBoard(drive, superstructure);
        }
        return instance;
    }

    private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Trigger overrideTrigger = driver.rightTrigger(0.1);
	//private OverrideBehavior overrideBehavior = OverrideBehavior.CORAL_SCORE_L4;

	private Trigger rightBumper = driver.rightBumper();

	// public static enum OverrideBehavior {
	// 	NET_SCORE(() -> superstructure.netScore().andThen(superstructure.tuck())),
	// 	PROCESSOR_SCORE(
	// 			() -> superstructure.processorScore().andThen(superstructure.tuckAfterProcessor())),
	// 	ALGAE_HOLD(() -> superstructure.algaeStow()),
	// 	CORAL_SCORE_L1(() -> superstructure.softCoralScore()),
	// 	CORAL_SCORE_L2(() -> superstructure.coralScore(Level.L2)),
	// 	CORAL_SCORE_L3(() -> superstructure.coralScore(Level.L3)),
	// 	CORAL_SCORE_L4(() -> superstructure.coralScore(Level.L4)),
	// 	TUCK(() -> superstructure.tuck()),
	// 	NONE(() -> Commands.none());

	// 	public final Supplier<Command> action;

	// 	private OverrideBehavior(Supplier<Command> overrideAction) {
	// 		action = overrideAction;
	// 	}
	// }

	// public Command setOverrideBehavior(OverrideBehavior behavior) {
	// 	return Commands.runOnce(() -> overrideBehavior = behavior);
	// }

	public void configureBindings(Drive drive, Superstructure superstructure) {
		driver.start()
				.onTrue(Commands.runOnce(
								() -> drive.getDrivetrain().seedFieldCentric(), drive)
						.ignoringDisable(true));

		driver.back()
				.onTrue(Commands.runOnce(
								() -> drive.getDrivetrain().resetPose(FieldLayout.kAprilTagMap.getTagPose(31).get().toPose2d()), drive)
						.ignoringDisable(true));
		driverControls();
		//debugControls();
	}

	// public OverrideBehavior getOverrideBehavior() {
	// 	return overrideBehavior;
	// }

	// public void bringupControls() {
	// 	operator.a().onTrue(Commands.runOnce(() -> Detection.mInstance.setPipeline(DetectionConstants.kAutoPipeline)));
	// 	operator.b().onTrue(Commands.runOnce(() -> Detection.mInstance.setPipeline(DetectionConstants.kTelePipeline)));
	// }

	public void driverControls() {
		Superstructure s = superstructure;

		// MISC ###############################################################################

		driver.a().whileTrue(s.spit()).onFalse(s.setState(Superstructure.State.DEPLOYED));

 		driver.leftBumper().onTrue(s.tuck());

 		// INTAKING ###############################################################################

 		driver.leftTrigger(0.1)
 				.whileTrue(
 						s.runIntakeIfDeployed())
						.onFalse(Commands.either(s.setState(State.SHOOTING), s.setState(State.DEPLOYED), () -> s.getState() == State.SHOOTINTAKE));
 						//.withName("Deploy and/or Intake"));

 		driver.x().whileTrue(
						Commands.sequence(
							Commands.runOnce(() -> s.maintainHeadingEpsilon = 0.00),
							s.shootWhenReady()
							.withName("Shooting").finallyDo(() -> superstructure.maintainHeadingEpsilon = 0.25)).withName("Shooting")
		).onFalse(Commands.either(s.setState(State.INTAKING), s.setState(State.DEPLOYED), () -> s.getState() == State.SHOOTINTAKE));

		driver.b().whileTrue(s.climb()).onFalse(s.setState(Superstructure.State.CLIMBING));

		driver.povLeft().onTrue(s.toggleSOTM().withName("SOTM Toggle"));

		driver.povDown().whileTrue(s.driveBrake().withName("Brake"));

		driver.povRight().onTrue((Commands.runOnce(() -> s.headingLockToggle = !s.headingLockToggle)));


 	}

	// public Command shootingSetOverrideBehavior(Trigger button) {
	// 	return setOverrideBehavior(OverrideBehavior.ALGAE_HOLD)
	// 			.onlyWhile(button)
	// 			.until(overrideTrigger);
	// }


// // 	private void debugControls() {
// // 		Superstructure s = superstructure;

// // 		operator.y().onTrue(s.stationIntakeToHold());
// // 		operator.a()
// // 				.onTrue(CoralRollers.mInstance.setpointCommand(CoralRollers.START))
// // 				.onFalse(CoralRollers.mInstance.setpointCommand(CoralRollers.IDLE));

// // 		operator.b()
// // 				.onTrue(Climber.mInstance
// // 						.setpointCommand(Climber.JOG_UP)
// // 						.andThen(() -> Climber.mInstance.useSoftLimits(false)))
// // 				.onFalse(Climber.mInstance
// // 						.setpointCommand(Climber.HOLD)
// // 						.andThen(() -> Climber.mInstance.useSoftLimits(true)));
// // 		operator.x()
// // 				.onTrue(Climber.mInstance
// // 						.setpointCommand(Climber.JOG_DOWN)
// // 						.andThen(() -> Climber.mInstance.useSoftLimits(false)))
// // 				.onFalse(Climber.mInstance
// // 						.setpointCommand(Climber.HOLD)
// // 						.andThen(() -> Climber.mInstance.useSoftLimits(true)));

// // 		operator.rightBumper()
// // 				.onTrue(Elevator.mInstance
// // 						.setpointCommand(Elevator.JOG_UP)
// // 						.andThen(() -> Elevator.mInstance.useSoftLimits(false)))
// // 				.onFalse(Elevator.mInstance
// // 						.setpointCommand(Elevator.HOLD_UP)
// // 						.andThen(() -> Elevator.mInstance.useSoftLimits(true)));
// // 		operator.rightTrigger(0.1)
// // 				.onTrue(Elevator.mInstance
// // 						.setpointCommand(Elevator.JOG_DOWN)
// // 						.andThen(() -> Elevator.mInstance.useSoftLimits(false)))
// // 				.onFalse(Elevator.mInstance
// // 						.setpointCommand(Elevator.HOLD_UP)
// // 						.andThen(() -> Elevator.mInstance.useSoftLimits(true)));

// // 		operator.povUp().onTrue(ClimberRollers.mInstance.setpointCommand(ClimberRollers.INTAKE));

// // 		operator.povDown().onTrue(ClimberRollers.mInstance.setpointCommand(ClimberRollers.IDLE));

// // 		operator.leftBumper()
// // 				.onTrue(new InstantCommand(() -> Pivot.mInstance.setCurrentPosition(
// // 								Pivot.mInstance.directCancoder.getPosition().getValue()))
// // 						.ignoringDisable(true));

// // 		operator.leftTrigger(0.1).onTrue(Elevator.mInstance.setpointCommand(Elevator.CLEAR_HIGH_HEIGHT));

// // 		operator.back()
// // 				.onTrue(Commands.sequence(
// // 						Commands.either(
// // 								Commands.none(),
// // 								AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.FAR_CLEAR),
// // 								() -> AlgaeDeploy.mInstance
// // 										.getPosition()
// // 										.isNear(AlgaeDeployConstants.kFarClearPosition, 0.1)),
// // 						Pivot.mInstance.setpointCommand(Pivot.JOG_POSITIVE)))
// // 				.onFalse(Pivot.mInstance.setpointCommand(Pivot.HOLD));

// // 		operator.start()
// // 				.onTrue(Commands.sequence(
// // 						Commands.either(
// // 								Commands.none(),
// // 								AlgaeDeploy.mInstance.setpointCommand(AlgaeDeploy.FAR_CLEAR),
// // 								() -> AlgaeDeploy.mInstance
// // 										.getPosition()
// // 										.isNear(AlgaeDeployConstants.kFarClearPosition, 0.1)),
// // 						Pivot.mInstance.setpointCommand(Pivot.JOG_NEGATIVE)))
// // 				.onFalse(Pivot.mInstance.setpointCommand(Pivot.HOLD));
// // 	}

	public Command rumbleCommand(Time duration) {
		return Commands.sequence(
						Commands.runOnce(() -> {
							setRumble(true);
						}),
						Commands.waitSeconds(duration.in(Units.Seconds)),
						Commands.runOnce(() -> {
							setRumble(false);
						}))
				.handleInterrupt(() -> {
					setRumble(false);
					;
				});
	}

	public void setRumble(boolean on) {
		ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}

// // 	public void configureSysIDTests() {
// // 		// Run SysId routines when holding back/start and X/Y.
// // 		// Note that each routine should be run exactly once in a single log.
// // 		driver.back()
// // 				.and(driver.y())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdDynamic(Direction.kForward));
// // 		driver.back()
// // 				.and(driver.x())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdDynamic(Direction.kReverse));
// // 		driver.start()
// // 				.and(driver.y())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdQuasistatic(Direction.kForward));
// // 		driver.start()
// // 				.and(driver.x())
// // 				.whileTrue(drive.getGeneratedDrive().sysIdQuasistatic(Direction.kReverse));

// // 		// Reset the field-centric heading on left bumper press
// // 		driver.leftBumper().onTrue(drive.getGeneratedDrive().runOnce(() -> drive
// // 				.getGeneratedDrive()
// // 				.seedFieldCentric()));
// // 	}

// // 	public void configureModulePointing() {
// // 		driver.a().whileTrue(drive.getGeneratedDrive().applyRequest(() -> brake));
// // 		driver.b()
// // 				.whileTrue(drive
// // 						.getGeneratedDrive()
// // 						.applyRequest(() ->
// // 								point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
// // 	}
}