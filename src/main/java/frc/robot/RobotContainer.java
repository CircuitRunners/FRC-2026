// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.drive.AutoAlignToReefCommand;
import frc.lib.drive.DriveMaintainingHeading;
import frc.lib.drive.DriveToPose;
import frc.lib.drive.FollowSyncedPIDToPose;
import frc.lib.drive.FollowTrajectoryCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.FieldLayout.Level;
import frc.lib.drive.DriveMaintainingHeading.DriveHeadingState;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.ControlBoardConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

@Logged
public class RobotContainer {
    private final Drive drive = new Drive();
    private final Superstructure superstructure = new Superstructure(drive);

    private final ControlBoard controlBoard = ControlBoard.getInstance(drive, superstructure);

    

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final Vision vision = new Vision(
        drive.getDrivetrain().getVisionConsumer(),
        (RobotBase.isSimulation())
        ? new VisionIOPhotonVisionSim(VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera, drive::getPose)
        : new VisionIOPhotonVision(VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera, drive::getPose)
        // (RobotBase.isSimulation())
        // ? new VisionIOPhotonVisionSim(VisionConstants.rightCameraName, VisionConstants.robotToRightCamera, drive::getPose)
        // : new VisionIOPhotonVision(VisionConstants.rightCameraName, VisionConstants.robotToRightCamera, drive::getPose)
    );

    public RobotContainer() {
        //controlBoard.configureBindings(drive, superstructure);
        configureBindings();
    }

    private void configureBindings() {
        ControlBoardConstants.mDriverController.a().whileTrue(pidToPoseTest);
        ControlBoardConstants.mDriverController.leftTrigger().whileTrue(autoAlignToLeftBranch);
        ControlBoardConstants.mDriverController.rightTrigger().whileTrue(autoAlignToRightBranch);
        drive.setDefaultCommand(
            driveCommand
        );
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drive.getDrivetrain().setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drive.getDrivetrain().applyRequest(() ->
        //     driveRequest.withVelocityX(-ControlBoardConstants.mDriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-ControlBoardConstants.mDriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-ControlBoardConstants.mDriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drive.getDrivetrain().applyRequest(() -> idle).ignoringDisable(true)
        //);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // ControlBoardConstants.mDriverController.back().and(ControlBoardConstants.mDriverController.y()).whileTrue(drive.getDrivetrain().sysIdDynamic(Direction.kForward));
        // ControlBoardConstants.mDriverController.back().and(ControlBoardConstants.mDriverController.x()).whileTrue(drive.getDrivetrain().sysIdDynamic(Direction.kReverse));
        // ControlBoardConstants.mDriverController.start().and(ControlBoardConstants.mDriverController.y()).whileTrue(drive.getDrivetrain().sysIdQuasistatic(Direction.kForward));
        // ControlBoardConstants.mDriverController.start().and(ControlBoardConstants.mDriverController.x()).whileTrue(drive.getDrivetrain().sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        ControlBoardConstants.mDriverController.start().onTrue(drive.getDrivetrain().runOnce(() -> drive.getDrivetrain().seedFieldCentric()));
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }


    private final DriveMaintainingHeading driveCommand = 
        new DriveMaintainingHeading(drive, () -> ControlBoardConstants.mDriverController.getLeftY(), () -> ControlBoardConstants.mDriverController.getLeftX(), () -> -ControlBoardConstants.mDriverController.getRightX());

    private final Command syncedPIDToPoseTest =
        new FollowSyncedPIDToPose(drive, superstructure, new Pose2d(5.0, 2.8, Rotation2d.fromDegrees(270)), Level.NET);
    
    private final Command pidToPoseTest =
        new PIDToPoseCommand(drive, superstructure, new Pose2d(2.0, 0, Rotation2d.fromDegrees(0)));

    private final Command autoAlignToLeftBranch =
        new AutoAlignToReefCommand(drive, superstructure, true);
    
    private final Command autoAlignToRightBranch =
        new AutoAlignToReefCommand(drive, superstructure, false);


    
    


}
