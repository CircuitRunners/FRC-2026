// MIT License

// Copyright (c) 2025-2026 Littleton Robotics

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
package frc.robot.shooting;

import frc.lib.util.FieldLayout;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCalculator {
	private Drive drive;
	private static ShotCalculator instance;
	public ShotCalculator(Drive drive) {
		this.drive = drive;
	}

	private Rotation2d heading;
	private double hoodAngle = Double.NaN;

	public static ShotCalculator getInstance(Drive drive) {
		if (instance == null) instance = new ShotCalculator(drive);
		return instance;
	}

	public record ShootingParameters(
		boolean isValid,
		Rotation2d heading,
		double hoodAngle,
		double flywheelSpeed) {}
	  
	private ShootingParameters latestParameters = null;

  	private static double minDistance;
  	private static double maxDistance;
  	private static double phaseDelay;

	static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;
  	}

	public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = drive.getPose();
    ChassisSpeeds robotRelativeVelocity = drive.getRobotRelativeChassisSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

	// Calculate distance from turret to target
    Translation2d target =
        FieldLayout.handleAllianceFlip(FieldLayout.blueHubCenter, RobotConstants.isRedAlliance);
    Pose2d shooterPosition = estimatedPose.transformBy(ShooterConstants.robotToShooter);
    double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

	// Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drive.getFieldRelativeChassisSpeeds();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double shooterVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (ShooterConstants.robotToShooter.getY() * Math.cos(robotAngle)
                    - ShooterConstants.robotToShooter.getX() * Math.sin(robotAngle));
    double shooterVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (ShooterConstants.robotToShooter.getX() * Math.cos(robotAngle)
                    - ShooterConstants.robotToShooter.getY() * Math.sin(robotAngle));

	// Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = shooterPosition;
    double lookaheadTurretToTargetDistance = shooterToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = getTimeOfFlightForShot(lookaheadTurretToTargetDistance);
      double offsetX = shooterVelocityX * timeOfFlight;
      double offsetY = shooterVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              shooterPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

	// Calculate parameters accounted for imparted velocity
    heading = target.minus(lookaheadPose.getTranslation()).getAngle();
    hoodAngle = getHoodSetpointForShot(lookaheadTurretToTargetDistance);
    latestParameters =
        new ShootingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
			heading,
            hoodAngle,
            getShooterSetpointForShot(lookaheadTurretToTargetDistance));

    // Log calculated values
    // Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    // Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

    return latestParameters;
	}

    public void clearShootingParameters() {
    latestParameters = null;
  }

	// interpolates distance to target for shooter setpoint along regression
	private static double getShooterSetpointForShot(double range) {
		return RegressionMaps.kFlywheelAutoAimMap.get(range);
	}

	// interpolates distance to target for hood setpoint along regression
	private static double getHoodSetpointForShot(double range) {
		return RegressionMaps.kHoodAutoAimMap.get(range);
	}

    // interpolates distance to target for time of flight along regression
	private static double getTimeOfFlightForShot(double range) {
		return RegressionMaps.kTimeOfFlightMap.get(range);
	}
}