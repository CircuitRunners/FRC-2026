package frc.robot.shooting;

import frc.lib.util.FieldLayout;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.lib.logging.LogUtil;
import frc.lib.util.team254_2022.InterpolatingDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingUtil {
	public static final boolean kEnableShootOnMove = true;
	public static final double kToFFactor = 0.2;

	private static final Translation2d kHubTarget = FieldLayout.blueHubCenter;

	/**
	 * Get speaker shot parameters
	 * @param robot_pose Pose of the robot
	 * @param is_red_alliance
	 * @return Array of length 4 containing distance to target, hood angle, shooter rpm, and drivetrain angle
	 */
	public static double[] getSpeakerShotParameters(
			Pose2d robot_pose, Transform2d robot_velocity, boolean is_red_alliance) {
		Translation2d target = FieldLayout.handleAllianceFlip(kHubTarget, is_red_alliance);
		Translation2d robot_to_target =
				target.minus(robot_pose.getTranslation());

		LogUtil.recordTranslation2d(
				"LookaheadPose",
				robot_pose.getTranslation().plus(new Translation2d(robot_velocity.getX(), robot_velocity.getY())));

		double yaw = robot_to_target.getAngle().getDegrees();
		double dist = robot_to_target.getNorm();
		double uncompensated_range = dist;

		SmartDashboard.putNumber("FiringParams/Uncomped Dist", dist);

		double shooter_setpoint, hood_setpoint;
		Rotation2d target_drive_heading = Rotation2d.fromDegrees(yaw + 180.0);
		shooter_setpoint = getShooterSetpointForShot(dist);
		hood_setpoint = getHoodSetpointForShot(dist);
		SmartDashboard.putNumber("FiringParams/Y Dist", robot_to_target.getY());

		return new double[] {dist, hood_setpoint, shooter_setpoint, target_drive_heading.getDegrees()};
	}

	// interpolates distance to target for shooter setpoint along regression
	private static double getShooterSetpointForShot(double range) {
		return RegressionMaps.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
	}

	// interpolates distance to target for hood setpoint along regression
	private static double getHoodSetpointForShot(double range) {
		return RegressionMaps.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
	}
}