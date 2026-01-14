package frc.lib.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.MathHelpers;
import frc.robot.auto.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class DriveToPose extends Command{
    private ProfiledPIDController driveController;


    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    AutoConstants.kPThetaController,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            DriveConstants.kDriveMaxAngularRate,
                            DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                    0.02);
    

    private Drive drive;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.0, ffMaxRadius = 0.1;
    private Pose2d targetLocation;
    
    public DriveToPose (
        Drive drive, 
        Pose2d targetLocation, 
        double constraintFactor) {
            this.drive = drive;
            this.targetLocation = targetLocation;
            this.driveController = 
                new ProfiledPIDController(
                            AutoConstants.kPXYController,
                            0.0,
                            0.0,
                            new TrapezoidProfile.Constraints(
                                    DriveConstants.kDriveMaxSpeed * constraintFactor,
                                    DriveConstants.kMaxAccelerationMetersPerSecondSquared
                                            * constraintFactor),
                            0.02);
            addRequirements(drive);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();

        driveController.reset(
        //distance from initial position to target location in meters
            currentPose.getTranslation().getDistance(targetLocation.getTranslation()),
             Math.min(
                        //clamps velocity if robot is moving away from target location (positive x velocity)
                        0.0,
                        -new Translation2d(
                                        drive.getFieldRelativeChassisSpeeds()
                                                .vxMetersPerSecond,
                                        drive.getFieldRelativeChassisSpeeds()
                                                .vyMetersPerSecond)
                                .rotateBy(
                                        targetLocation
                                                .getTranslation()
                                                .minus(
                                                        drive
                                                                .getPose()
                                                                .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                //current orientation in radians
                currentPose.getRotation().getRadians(),
                //robot angular velocity
                drive.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(2.0));
    }

    @Override
    public void execute() {
        
        Pose2d currentPose = drive.getPose();

        double currentDistance = currentPose.getTranslation().getDistance(targetLocation.getTranslation());
        //Math.clamp keeps ffScaler between 0 and 1
        double ffScaler = MathUtil.clamp(
                (currentDistance-ffMinRadius)/ ( ffMaxRadius - ffMinRadius),
                0.0,
                1.0);

        driveErrorAbs = currentDistance;
        //adds ideal velocity from motion profile to PID correction for velocity command
        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScaler
                        + driveController.calculate(driveErrorAbs, 0.0);


        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;


        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScaler
                        + thetaController.calculate(
                                currentPose.getRotation().getRadians(),
                                targetLocation.getRotation().getRadians());

        thetaErrorAbs =
        Math.abs(
                currentPose.getRotation().minus(targetLocation.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        
        var driveVelocity = MathHelpers.pose2dFromRotation(
                                currentPose
                                        .getTranslation()
                                        .minus(targetLocation.getTranslation())
                                        .getAngle())
                        .transformBy(
                                MathHelpers.transform2dFromTranslation(
                                        new Translation2d(driveVelocityScalar, 0.0)))
                        .getTranslation();
           
        

        drive.getDrivetrain().setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        driveVelocity.getX(),
                                        driveVelocity.getY(),
                                        thetaVelocity,
                                        currentPose.getRotation())));

    }

    @Override
    public void end (boolean interrupted) {
        drive.getDrivetrain().applyRequest(() -> new SwerveRequest.ApplyRobotSpeeds());
    }

    @Override
    public boolean isFinished() {
        return targetLocation.equals(null)
                || (driveController.atGoal() && thetaController.atGoal());
    }



}

