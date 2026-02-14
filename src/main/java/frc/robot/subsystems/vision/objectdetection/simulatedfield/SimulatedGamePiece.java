package frc.robot.subsystems.vision.objectdetection.simulatedfield;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;

public class SimulatedGamePiece {
    private static final ArrayList<SimulatedGamePiece> SIMULATED_GAME_PIECES = new ArrayList<>();
    private static final ArrayList<Rotation2d> CURRENT_SPINDEXER_RELATIVE_OCCUPIED_ROTATIONS = new ArrayList<>();

    private Translation3d fieldRelativePosition;
    private boolean isIndexed = true;
    private Rotation2d spindexerRelativeRotation;
    private Translation3d unindexedRobotRelativeStorePosition;

    public SimulatedGamePiece(double startingPoseXMeters, double startingPoseYMeters) {
        SimulatedGamePieceConstants.GamePieceType gamePieceType = SimulatedGamePieceConstants.GamePieceType.FUEL;
        fieldRelativePosition = new Translation3d(startingPoseXMeters, startingPoseYMeters, gamePieceType.originPointHeightOffGroundMeters);
        SIMULATED_GAME_PIECES.add(this);
    }

    public static ArrayList<SimulatedGamePiece> getSimulatedGamePieces() {
        return SIMULATED_GAME_PIECES;
    }

    public static ArrayList<SimulatedGamePiece> getUnheldGamePieces() {
        final ArrayList<SimulatedGamePiece> unheldGamePieces = new ArrayList<>(SIMULATED_GAME_PIECES);
        unheldGamePieces.removeIf(SimulatedGamePiece::isHeld);

        return unheldGamePieces;
    }

    public void updatePosition(Translation3d fieldRelativePosition) {
        this.fieldRelativePosition = fieldRelativePosition;
    }

    public Translation3d getPosition() {
        return fieldRelativePosition;
    }

    void release() {
        unindexedRobotRelativeStorePosition = null;
    }

    double getDistanceFromPositionMeters(Translation3d position) {
        return fieldRelativePosition.getDistance(position);
    }


    static void logAll() {
        //log dis
       // Logger.recordOutput("Poses/GamePieces/Fuel", getSimulatedFuelAsPoseArray());
    }

    private static Pose3d[] getSimulatedFuelAsPoseArray() {
        final Pose3d[] poses = new Pose3d[SimulatedGamePiece.SIMULATED_GAME_PIECES.size()];
        for (int i = 0; i < poses.length; i++)
            poses[i] = new Pose3d(SimulatedGamePiece.SIMULATED_GAME_PIECES.get(i).getPosition(), new Rotation3d());
        return poses;
    }

    private boolean isHeld() {
        return spindexerRelativeRotation != null || unindexedRobotRelativeStorePosition != null;
    }

    
    //WARNING: 100% vibe coded from this point
    private Translation3d calculateRobotRelativeStorePosition() {
        final Translation3d cornerA = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_START;
        final Translation3d cornerB = SimulatedGamePieceConstants.ROBOT_RELATIVE_HELD_UNINDEXED_FUEL_BOUNDING_BOX_END;

        // Calculate radius to ensure the center is far enough from walls
        final double fuelRadius = SimulatedGamePieceConstants.GamePieceType.FUEL.originPointHeightOffGroundMeters;

        // 1. Normalize the bounds (finding the true min and max for X, Y, Z)
        double minX = Math.min(cornerA.getX(), cornerB.getX());
        double maxX = Math.max(cornerA.getX(), cornerB.getX());

        double minY = Math.min(cornerA.getY(), cornerB.getY());
        double maxY = Math.max(cornerA.getY(), cornerB.getY());

        double minZ = Math.min(cornerA.getZ(), cornerB.getZ());
        double maxZ = Math.max(cornerA.getZ(), cornerB.getZ());

        // 2. Inset the boundaries by the fuel radius
        // This prevents the fuel from poking out of the box
        minX += fuelRadius;
        maxX -= fuelRadius;

        minY += fuelRadius;
        maxY -= fuelRadius;

        minZ += fuelRadius;
        maxZ -= fuelRadius;

        // 3. Safety Check: If the box is smaller than the fuel diameter, it can't fit
        if (minX >= maxX || minY >= maxY || minZ >= maxZ) {
            return null;
        }

        // 4. Generate the random position within the safe (shrunk) bounds
        double randomX = minX + (Math.random() * (maxX - minX));
        double randomY = minY + (Math.random() * (maxY - minY));
        double randomZ = minZ + (Math.random() * (maxZ - minZ));

        return new Translation3d(randomX, randomY, randomZ);
    }


    private boolean isAngleInSector(double angle, double start, double end) {
        double normalizedStart = MathUtil.angleModulus(start);
        double normalizedEnd = MathUtil.angleModulus(end);
        double normalizedAngle = MathUtil.angleModulus(angle);

        if (normalizedStart <= normalizedEnd) {
            return normalizedAngle >= normalizedStart && normalizedAngle <= normalizedEnd;
        } else {
            return normalizedAngle >= normalizedStart || normalizedAngle <= normalizedEnd;
        }
    }
}