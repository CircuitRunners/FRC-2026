package frc.lib.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class DriveMaintainingHeading extends Command{
    protected Drive mDrivetrain;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mTurnSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;
    private DriveHeadingState mDriveMode = DriveHeadingState.BARGE_HEADING;

    public enum DriveHeadingState {
        NO_HEADING,
        MAINTAIN_HEADING,
        BARGE_HEADING,
        REEF_HEADING,
        PROCESSOR_HEADING;

        public String toString() {
            return name();
        }
    }

    private final SendableChooser<DriveHeadingState> headingStateChooser = new SendableChooser<>();
    private DriveHeadingState headingState = DriveHeadingState.NO_HEADING;
    
    private final SwerveRequest.FieldCentric driveNoHeading = 
        new SwerveRequest.FieldCentric()
            .withDeadband(
                DriveConstants.kDriveMaxSpeed * DriveConstants.kDriveJoystickDeadband
            )
            .withRotationalDeadband(
                DriveConstants.kDriveMaxAngularRate * DriveConstants.kSteerJoystickDeadband
            )
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading = 
        new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.kDriveMaxSpeed * DriveConstants.kDriveJoystickDeadband)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public DriveMaintainingHeading(
        Drive drivetrain,
        DoubleSupplier throttle,
        DoubleSupplier strafe,
        DoubleSupplier turn
        ) 
    {
        mDrivetrain = drivetrain;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

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

        headingStateChooser.setDefaultOption("Reef Heading", DriveHeadingState.REEF_HEADING);
        headingStateChooser.addOption("Barge Heading", DriveHeadingState.BARGE_HEADING);
        headingStateChooser.addOption("Processor Heading", DriveHeadingState.PROCESSOR_HEADING);
        headingStateChooser.addOption("Manual Heading", DriveHeadingState.NO_HEADING);
        SmartDashboard.putData("Heading State Selector", headingStateChooser);
    }

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double throttleFieldFrame = RobotConstants.isRedAlliance ? throttle : -throttle;
        double strafeFieldFrame = RobotConstants.isRedAlliance ? strafe : -strafe;
        //if (Robot.isSimulation()) {throttleFieldFrame = -throttleFieldFrame; strafeFieldFrame = -strafeFieldFrame;}
        updateHeadingState();
        mDriveMode = headingState;

        if (Math.abs(turnFieldFrame) > DriveConstants.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }

        if (Math.abs(turnFieldFrame) > DriveConstants.kSteerJoystickDeadband
                || (MathHelpers.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
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

            if (mDriveMode == DriveHeadingState.REEF_HEADING) {
                double targetAngle = FieldLayout.Branch.getClosestFace(
                    mDrivetrain.getPose(), 
                    RobotConstants.isRedAlliance)
                    .rotation.getDegrees();

                mDrivetrain.getDrivetrain().setControl(
                    driveWithHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withTargetDirection(
                                RobotConstants.isRedAlliance
                                            ? Util.flipRedBlue(Rotation2d.fromDegrees(targetAngle))
                                            : Rotation2d.fromDegrees(targetAngle)));
                mHeadingSetpoint = Optional.of(mDrivetrain.getPose().getRotation());
            } else if (mDriveMode == DriveHeadingState.BARGE_HEADING) {
                double targetAngle =
                        mDrivetrain.getPose().getX()
                                        > FieldLayout.kFieldLength.in(Units.Meters) / 2.0
                                ? Math.PI
                                : 0;

                if (Math.abs(
                    mDrivetrain.getPose().getX() - FieldLayout.kFieldLength.in(Units.Meters) / 2.0) > 1.0
                ) {
                    mHeadingSetpoint = Optional.of(new Rotation2d(targetAngle));
                }
                mDrivetrain.getDrivetrain().setControl(driveWithHeading
                    .withVelocityX(throttleFieldFrame)
                    .withVelocityY(strafeFieldFrame)
                    .withTargetDirection(mHeadingSetpoint.get()));

            } else if (mDriveMode == DriveHeadingState.PROCESSOR_HEADING) {
                double targetAngle =
                    RobotConstants.isRedAlliance
                                ? Math.PI / 2
                                : -Math.PI / 2;
                if (mDrivetrain.onOpponentSide()) {
                    targetAngle += Math.PI;
                }
                mHeadingSetpoint = Optional.of(new Rotation2d(targetAngle));
                mDrivetrain.getDrivetrain().setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));
            }
            else {
                mDrivetrain.getDrivetrain().setControl(
                        driveWithHeading
                                .withVelocityX(throttleFieldFrame)
                                .withVelocityY(strafeFieldFrame)
                                .withTargetDirection(mHeadingSetpoint.get()));
            }

        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void updateHeadingState() {
        headingState = headingStateChooser.getSelected();
    }
   
}
