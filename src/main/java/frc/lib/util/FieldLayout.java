package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in inches, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Inches</b> <br> as used in the game manual and field drawings
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
    public static Distance kFieldLength = Units.Inches.of(651.22);
	public static Distance kFieldWidth = Units.Inches.of(317.69);
	public static Distance kAllianceZoneX = Units.Inches.of(158.61);
    public static AprilTagFieldLayout kAprilTagMap =
			AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
	public static Distance kAprilTagWidth = Units.Inches.of(6.5);

    public static final Translation2d blueHubCenter = new Translation2d(Units.Inches.of(182.11), kFieldWidth.div(2.0));
	public static final Pose2d blueOutpostPose = kAprilTagMap.getTagPose(13).get().toPose2d();
	public static final Translation2d blueDepotCenter = new Translation2d(Units.Inches.of(13.5), kFieldWidth.div(2.0).plus(Units.Inches.of(75.93)));
	public static final Translation2d kInnerBottomTrenchEdge = new Translation2d(Units.Inches.of(blueHubCenter.getX()), Units.Inches.of(50.34));
	public static final Translation2d kInnerTopTrenchEdge = new Translation2d(Units.Inches.of(blueHubCenter.getX()), kFieldWidth.minus(Units.Inches.of(50.34)));



	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = rotateAboutCenter(blue_pose, Rotation2d.k180deg);
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.rotateAround(
					new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), Rotation2d.k180deg);
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.plus(Rotation2d.k180deg);
		}
		return blue_rotation;
	}

	public static Distance distanceFromAllianceWall(Distance x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength.minus(x_coordinate);
		}
		return x_coordinate;
	}

	public static boolean nearTrench(Pose2d pose, boolean is_red_alliance) {
		Distance x = pose.getMeasureX();
    	Distance y = pose.getMeasureY();

    	boolean inTrenchY =
            y.lte(kInnerBottomTrenchEdge.getMeasureY())
         || y.gte(kInnerTopTrenchEdge.getMeasureY());

    	Distance distFromWall = distanceFromAllianceWall(x, is_red_alliance);

    	boolean nearTrenchX =
            distFromWall.isNear(handleAllianceFlip(blueHubCenter, is_red_alliance).getMeasureX(), Units.Inches.of(47.00 / 2));

    	return nearTrenchX && inTrenchY;
	}

	/** Changes heading so the robot doesn't get stuck in the trench at certain angles
	 * @param rotation current rotation
	 * @return adjusted rotation
	 */
	public static Rotation2d clampAwayFromTrench(Rotation2d rotation) {
		double deg = rotation.getDegrees();

		if (deg >= 40 && deg <= 70) {
			return Rotation2d.fromDegrees((deg - 40 < 70 - deg) ? 40 : 70);
		}

		if (deg >= -140 && deg <= -110) {
			return Rotation2d.fromDegrees((deg - (-140) < -110 - deg) ? -140 : -110);
		}

		if (deg >= 130 && deg <= 160) {
			return Rotation2d.fromDegrees((deg - 130 < 160 - deg) ? 130 : 160);
		}

		if (deg >= -50 && deg <= -20) {
			return Rotation2d.fromDegrees((deg - (-50) < -20 - deg) ? -50 : -20);
		}

		return rotation;
	}





	public static Translation2d mirrorAboutX(Translation2d t, Distance xValue) {
		return new Translation2d(xValue.in(Units.Meters) + (xValue.in(Units.Meters) - t.getX()), t.getY());
	}

	public static Translation2d mirrorAboutY(Translation2d t, Distance yValue) {
		return new Translation2d(t.getX(), yValue.in(Units.Meters) + (yValue.in(Units.Meters) - t.getY()));
	}

	public static Rotation2d mirrorAboutX(Rotation2d r) {
		return new Rotation2d(-r.getCos(), r.getSin());
	}

	public static Rotation2d mirrorAboutY(Rotation2d r) {
		return new Rotation2d(r.getCos(), -r.getSin());
	}

	public static Pose2d mirrorAboutX(Pose2d p, Distance xValue) {
		return new Pose2d(mirrorAboutX(p.getTranslation(), xValue), mirrorAboutX(p.getRotation()));
	}

	public static Pose2d mirrorAboutY(Pose2d p, Distance yValue) {
		return new Pose2d(mirrorAboutY(p.getTranslation(), yValue), mirrorAboutY(p.getRotation()));
	}

	public static Pose2d rotateAboutPose(Pose2d startPose, Translation2d point, Rotation2d rotation) {
		return new Pose2d(
				startPose.getTranslation().rotateAround(point, rotation),
				startPose.getRotation().plus(rotation));
	}

	public static Pose2d rotateAboutCenter(Pose2d startPose, Rotation2d rotation) {
		return rotateAboutPose(startPose, new Translation2d(kFieldLength.div(2.0), kFieldWidth.div(2.0)), rotation);
	}

}