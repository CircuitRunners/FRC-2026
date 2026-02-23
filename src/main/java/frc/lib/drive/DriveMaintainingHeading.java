package frc.lib.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldLayout;
import frc.lib.util.MathHelpers;
import frc.lib.util.Util;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.shooting.ShotCalculator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.Superstructure.State;

public class DriveMaintainingHeading extends Command{
    public DriveMaintainingHeading(
        Drive drivetrain,
        Superstructure superstructure,
        DoubleSupplier throttle,
        DoubleSupplier strafe,
        DoubleSupplier turn,
        DoubleSupplier epsilon
        ) 
    {
        mDrivetrain = drivetrain;
        mSuperstructure = superstructure;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;
        mEpsilonSupplier = epsilon;

        driveWithHeading.HeadingController.setPID(
            DriveConstants.kHeadingControllerP,
            DriveConstants.kHeadingControllerI,
            DriveConstants.kHeadingControllerD
        );

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    protected Drive mDrivetrain;
    protected Superstructure mSuperstructure;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mTurnSupplier;
    public final DoubleSupplier mEpsilonSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;
    
    private final SwerveRequest.FieldCentric driveNoHeading = 
        new SwerveRequest.FieldCentric()
            .withDeadband(
                DriveConstants.kDriveMaxSpeed * 0.05
            )
            .withRotationalDeadband(
                DriveConstants.kDriveMaxAngularRate * DriveConstants.kSteerJoystickDeadband
            )
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading = 
        new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.kDriveMaxSpeed * 0.15)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);


    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double epsilon = mEpsilonSupplier.getAsDouble();
        double throttleFieldFrame = RobotConstants.isRedAlliance ? throttle : -throttle;
        double strafeFieldFrame = RobotConstants.isRedAlliance ? strafe : -strafe;

        if (Math.abs(turnFieldFrame) > DriveConstants.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }

        if (Math.abs(turnFieldFrame) > DriveConstants.kSteerJoystickDeadband
                || (MathHelpers.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), epsilon)
                        && Math.abs(
                                        mDrivetrain.getRobotRelativeChassisSpeeds()
                                                .omegaRadiansPerSecond)
                                > Math.toRadians(10))) {
            
            mDrivetrain.getDrivetrain().setControl(
                    (driveNoHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withRotationalRate(
                                    turnFieldFrame
                                            * DriveConstants.kDriveMaxAngularRate)));
            mHeadingSetpoint = Optional.empty();
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint =
                        Optional.of(mDrivetrain.getPose().getRotation());
            }
            // dont get stuck in trench
            if (mSuperstructure.nearTrench && mSuperstructure.getState() != State.SHOOTING) {
                Rotation2d targetAngle = FieldLayout.clampAwayFromTrench(mDrivetrain.getRotation());
                mHeadingSetpoint = Optional.of(targetAngle);

                mDrivetrain.getDrivetrain().setControl(
                    driveWithHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withTargetDirection(
                                mHeadingSetpoint.get()
                            )
                );
            } else if (//FieldLayout.distanceFromAllianceWall(Units.Meters.of(mDrivetrain.getPose().getX()), RobotConstants.isRedAlliance).lte(FieldLayout.kAllianceZoneX.minus(Units.Inches.of(14)))
                     mSuperstructure.shouldHeadingLock()) {
                Rotation2d targetAngle = mSuperstructure.headingSetpoint;

                mDrivetrain.getDrivetrain().setControl(
                    driveWithHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withTargetDirection(
                                targetAngle
                            )
                );
                mHeadingSetpoint = 
                        Optional.of(mDrivetrain.getPose().getRotation());
            }
            
            else {
                mDrivetrain.getDrivetrain().setControl(
                        driveNoHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withRotationalRate(
                                    turnFieldFrame
                                            * DriveConstants.kDriveMaxAngularRate));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
