package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class TrajectoryHelpers {

    /** This computes the angle between the robot’s current position and the target’s position.
	 * Think of this as the direction you’d face if you pointed straight at the target.
     * @param drive
     * @param translation the translation of the reef branch
     * @return the angle between the robot’s current position and the target’s position
     */
    public static Rotation2d angleToScore(Drive drive, Translation2d translation) {
		Translation2d scoringTranslation = translation;
		return scoringTranslation
				.minus(drive.getPose().getTranslation())
				.getAngle();
	}

    /** Computs the angle to approach the robot at (useless because deadband is 0 so it always returns the branch rotation)
     * @param drive
     * @param translation the current robot translation
     * @param idealAngle the angle you want to face
     * @param deadband
     * @return the angle to approach
     */
    public static Rotation2d angleToApproach(Drive drive, Translation2d translation, Rotation2d idealAngle, Angle deadband) {
                Rotation2d currentAngle = angleToScore(drive, translation);
		double currentDegrees = currentAngle.getDegrees();
		double targetDegrees = idealAngle.getDegrees();

		if (idealAngle.getMeasure().isEquivalent(Units.Degrees.of(180))
				&& currentAngle.getMeasure().lte(Units.Degrees.of(0))) currentDegrees += 360.0;

		return Util.epsilonEquals(currentDegrees, targetDegrees, deadband.in(Units.Degrees))
				? Rotation2d.fromDegrees(currentDegrees)
				: idealAngle.plus(Rotation2d.fromDegrees(
						Math.signum(currentDegrees - targetDegrees) * deadband.in(Units.Degrees)));
	}

    /** returns the offsetted target pose from transformWantedGamepieceToDrivePose()
     * @param drive
     * @param endEffectorPose where you want the end effector to be (target pose)
     * @param idealApproachDeadband
     * @param level the level to score on
     * @return the offsetted pose
     */
    public static Pose2d getDriveTargetPose(Drive drive, Pose2d endEffectorPose, Angle idealApproachDeadband, Level level) {
		Rotation2d angle =
				angleToApproach(drive, endEffectorPose.getTranslation(), endEffectorPose.getRotation(), idealApproachDeadband);
		Pose2d transformedTargetPose = transformWantedGamepieceToDrivePose(
				new Pose2d(endEffectorPose.getTranslation(), angle),
				SuperstructureConstants.kElevatorCenterOffset.plus(
						SuperstructureConstants.getGamepieceOffsetFactor(level)));

		return transformedTargetPose;
	}

    /** Offsets the target pose by the coral's distance from the center of the robot
     * @param wantedFinalPose the pose you want to score at
     * @param offsetLength the distance the coral is from the center
     * @return the new offsetted pose
     */
    public static Pose2d transformWantedGamepieceToDrivePose(Pose2d wantedFinalPose, Distance offsetLength) {
		Rotation2d r = wantedFinalPose.getRotation();
		Distance x = wantedFinalPose.getMeasureX().minus(offsetLength.times(r.getCos()));
		Distance y = wantedFinalPose.getMeasureY().minus(offsetLength.times(r.getSin()));
		return new Pose2d(x, y, r);
	}
}
