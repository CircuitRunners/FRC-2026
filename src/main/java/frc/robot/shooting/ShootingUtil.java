package frc.robot.shooting;

import frc.lib.util.FieldLayout;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.lib.logging.LogUtil;
import frc.lib.util.team254_2022.InterpolatingDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
			Pose2d robot_pose, Twist2d robot_velocity, boolean is_red_alliance) {
		Translation2d target = FieldLayout.handleAllianceFlip(kHubTarget, is_red_alliance);
		Translation2d robot_to_target =
				target.minus(robot_pose.getTranslation());

		LogUtil.recordTranslation2d(
				"LookaheadPose",
				robot_pose.getTranslation().plus(new Translation2d(robot_velocity.dx, robot_velocity.dy)));

		double yaw = robot_to_target.getAngle().getDegrees();
		double dist = robot_to_target.getNorm();
		double uncompensated_range = dist;

		SmartDashboard.putNumber("FiringParams/Uncomped Dist", dist);

		if (kEnableShootOnMove) {
			double[] sotm_params = getAdjustedShootOnMoveParams(yaw, dist, robot_velocity);
			yaw = sotm_params[0];
			dist = sotm_params[1];
		}

		double shooter_setpoint, hood_setpoint;
		Rotation2d target_drive_heading = Rotation2d.fromDegrees(yaw + 180.0);
		shooter_setpoint = getShooterSetpointForShot(dist);
		hood_setpoint = getHoodSetpointForShot(dist);
		SmartDashboard.putNumber("FiringParams/Y Dist", robot_to_target.getY());

		return new double[] {dist, hood_setpoint, shooter_setpoint, target_drive_heading.getDegrees()};
	}

	/**
	 * Get new parameters given robot velocity
	 * @param uncompensated_yaw
	 * @param uncompensated_range
	 * @param robot_velocity
	 * @return Array of length 2 containing adjusted yaw and adjusted range
	 */
	private static double[] getAdjustedShootOnMoveParams(
			double uncompensated_yaw, double uncompensated_range, Twist2d robot_velocity) {
		Translation2d polar_velocity = new Translation2d(robot_velocity.dx, robot_velocity.dy)
				.rotateBy(Rotation2d.fromDegrees(uncompensated_yaw));
		double radial = polar_velocity.getX();
		double tangential = polar_velocity.getY();

		SmartDashboard.putNumber("FiringParams/Tangential", tangential);
		SmartDashboard.putNumber("FiringParams/Radial", radial);

		double shot_speed = uncompensated_range / kToFFactor - radial;
		shot_speed = Math.max(0.0, shot_speed);
		double yaw_adj = Units.radiansToDegrees(Math.atan2(-tangential, shot_speed));
		double range_adj = kToFFactor * Math.sqrt(Math.pow(tangential, 2) + Math.pow(shot_speed, 2));
		return new double[] {yaw_adj + uncompensated_yaw, range_adj};
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