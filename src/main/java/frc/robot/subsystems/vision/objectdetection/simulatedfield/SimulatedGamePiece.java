package frc.robot.subsystems.vision.objectdetection.simulatedfield;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class SimulatedGamePiece {
    private static final ArrayList<SimulatedGamePiece> SIMULATED_GAME_PIECES = new ArrayList<>();
    

    private Translation3d fieldRelativePosition;


    public SimulatedGamePiece(double startingPoseXMeters, double startingPoseYMeters) {
        SimulatedGamePieceConstants.GamePieceType gamePieceType = SimulatedGamePieceConstants.GamePieceType.FUEL;
        fieldRelativePosition = new Translation3d(startingPoseXMeters, startingPoseYMeters, gamePieceType.originPointHeightOffGroundMeters);
        SIMULATED_GAME_PIECES.add(this);
    }

    public static List<Pose2d> getPiecesAsPoses() {
        List<Pose2d> p = new ArrayList<>();
        for (Pose3d s : getSimulatedFuelAsPoseArray()) {
            p.add(new Pose2d(
                new Translation2d(s.getX(),
                s.getY()),
                Rotation2d.kZero
            ));
        }
        return p;
    }
    public static boolean removeGamePiece(SimulatedGamePiece g) {
        for (int i = 0; i < SIMULATED_GAME_PIECES.size(); i++) {
            if (SIMULATED_GAME_PIECES.get(i) == g) {
                SIMULATED_GAME_PIECES.remove(g);
                return true;
            }
        }
        return false;
    }
    
    public static ArrayList<SimulatedGamePiece> getSimulatedGamePieces() {
        return SIMULATED_GAME_PIECES;
    }

    public static ArrayList<SimulatedGamePiece> getUnheldGamePieces() {
        final ArrayList<SimulatedGamePiece> unheldGamePieces = new ArrayList<>(SIMULATED_GAME_PIECES);


        return unheldGamePieces;
    }

    public void updatePosition(Translation3d fieldRelativePosition) {
        this.fieldRelativePosition = fieldRelativePosition;
    }

    public Translation3d getPosition() {
        return fieldRelativePosition;
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