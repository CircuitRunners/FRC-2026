package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.drive.PIDToPosesCommand;
import frc.lib.util.FieldLayout;
import frc.lib.drive.DriveMaintainingHeading;
import frc.lib.drive.FollowNonstopTrajectory;
import frc.lib.drive.FollowTrajectoryCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intakeDeploy.IntakeDeploy;
import frc.robot.subsystems.intakeRollers.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.apriltag.Vision;
import frc.robot.subsystems.vision.apriltag.VisionConstants;
import frc.robot.subsystems.vision.apriltag.VisionIOPhotonVision;
import frc.robot.subsystems.vision.apriltag.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionConstants;
import frc.robot.subsystems.vision.objectdetection.ObjectPoseEstimator;
import frc.robot.subsystems.vision.objectdetection.objectdetectioncamera.ObjectDetectionCamera;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulatedGamePieceConstants;
import frc.robot.subsystems.vision.objectdetection.simulatedfield.SimulationFieldHandler;
import frc.robot.auto.AutoHelpers;
import frc.robot.auto.AutoModeSelector;
@Logged
public class RobotContainer {
    private final Drive drive = new Drive();
    private final Hood hood = new Hood();
    private final Vision vision = new Vision(
        drive.getDrivetrain().getVisionConsumer(),//,
        (RobotBase.isSimulation())
        ? new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose)
        : new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        (RobotBase.isSimulation())
        ? new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose)
        : new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose)
    );
    // public final ObjectPoseEstimator objectDetector = new ObjectPoseEstimator(
    //     drive,
    //     ObjectDetectionConstants.OBJECT_POSE_ESTIMATOR_DELETION_THRESHOLD_SECONDS,
    //     SimulatedGamePieceConstants.GamePieceType.FUEL,
    //     new ObjectDetectionCamera(
    //         drive,
    //         "ObjectDetection",
    //         ObjectDetectionConstants.cameraTransform
    //     )
    // );

    private final Shooter shooter = new Shooter();
    private final IntakeDeploy intakeDeploy = new IntakeDeploy();
    private final IntakeRollers intakeRollers = new IntakeRollers();
    private final Kicker kicker = new Kicker();
    private final Conveyor conveyor = new Conveyor();
    private final Climber climber = new Climber();
    private final Superstructure superstructure = new Superstructure(drive, vision, shooter, hood, intakeDeploy, intakeRollers, kicker, conveyor, climber , null);
    

    private final ControlBoard controlBoard = ControlBoard.getInstance(drive, shooter, hood, intakeDeploy, intakeRollers, kicker, conveyor, climber, superstructure);
    private final ShotCalculator shotCalculator = ShotCalculator.getInstance(drive);

    public ShotCalculator getShotCalculator() {
        return shotCalculator;
    }

    

    private AutoModeSelector mAutoModeSelector;
    private static String mPreviousAutoName;
    public AutoModeSelector getAutoModeSelector() {
        return mAutoModeSelector;
    }
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public RobotContainer() {
        SimulationFieldHandler.superstructure = this.superstructure;
        SimulationFieldHandler.drive = this.drive;
        controlBoard.configureBindings(drive, superstructure);
        configureBindings();
        RobotConstants.mAutoFactory = new AutoFactory(
				drive::getPose,
				drive.getDrivetrain()::resetPose,
				drive::followChoreoTrajectory,
                
				true,
				drive);

        // AutoHelpers.bindEventMarkers(RobotConstants.mAutoFactory)
        mAutoModeSelector = new AutoModeSelector(drive, superstructure, RobotConstants.mAutoFactory);
		mPreviousAutoName = mAutoModeSelector.getSelectedCommand().getName();
        SmartDashboard.putData("Auto Chooser", mAutoModeSelector.getAutoChooser());

        // pretty sure we don't need this, or we need to change it a bit cuz heading lock
        // RobotModeTriggers.autonomous()
		// 		.onFalse(Commands.runOnce(() -> drive.getDrivetrain().setControl(new SwerveRequest.ApplyFieldSpeeds()))
		// 				.ignoringDisable(true));
        // shooter.setDefaultCommand(shooter.trackTargetCommand(superstructure.shooterSetpoint));
        // hood.setDefaultCommand(hood.trackTargetCommand(superstructure.hoodSetpoint));

        // for (SubsystemBase s : new SubsystemBase[] {
		// 	intakeDeploy,
		// 	intakeRollers,
		// 	climber,
		// 	conveyor,
		// 	superstructure,
        //     kicker,
        //     shooter,
        //     hood
		// }) {
		// 	SmartDashboard.putData(s);
		// }
    }

    private void configureBindings() {
        // drive.setDefaultCommand(
        //     driveCommand
        // );

        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drive.getDrivetrain().setDefaultCommand(
            // Drivetrain will execute this command periodically
            drive.getDrivetrain().applyRequest(() ->
            driveRequest.withVelocityX(-ControlBoardConstants.mDriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-ControlBoardConstants.mDriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-ControlBoardConstants.mDriverController.getRightX() * MaxAngularRate).withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate*0.15) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drive.getDrivetrain().applyRequest(() -> idle).ignoringDisable(true)
        //);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drive.getDrivetrain().sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drive.getDrivetrain().sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drive.getDrivetrain().sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drive.getDrivetrain().sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //ControlBoardConstants.mDriverController.start().onTrue(drive.getDrivetrain().runOnce(() -> drive.getDrivetrain().seedFieldCentric()));
        ControlBoardConstants.mOperatorController.rightStick().onTrue(resetToVisionPose());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private final DriveMaintainingHeading driveCommand = 
        new DriveMaintainingHeading(drive, superstructure, () -> ControlBoardConstants.mDriverController.getLeftY(), () -> ControlBoardConstants.mDriverController.getLeftX(), () -> -ControlBoardConstants.mDriverController.getRightX(), () -> superstructure.maintainHeadingEpsilon);
    

    



    public Command resetToVisionPose() {
        return Commands.runOnce(() -> drive.getDrivetrain().resetPose(vision.getLatestVisionPose()));
    }

    

    

}