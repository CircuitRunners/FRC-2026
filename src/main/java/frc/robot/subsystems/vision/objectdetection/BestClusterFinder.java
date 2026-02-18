package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.*;

public class BestClusterFinder {

    public static class ClusterResult {
        public Translation2d center;
        public List<Translation2d> points;

        public ClusterResult(Translation2d center, List<Translation2d> points) {
            this.center = center;
            this.points = points;
        }
    }

    private static List<Translation2d> pointsInside(
            List<Translation2d> points,
            Translation2d center,
            double radius
    ) {
        List<Translation2d> inside = new ArrayList<>();

        for (Translation2d p : points) {
            if (p.getDistance(center) <= radius) {
                inside.add(p);
            }
        }

        return inside;
    }

    public static ClusterResult findBestCluster(
            List<Translation2d> points,
            double radius
    ) {

        if (points.isEmpty()) {
            return new ClusterResult(null, new ArrayList<>());
        }

        Translation2d bestCenter = null;
        List<Translation2d> bestPoints = new ArrayList<>();
        int bestCount = 0;

        int n = points.size();

        for (Translation2d p : points) {
            List<Translation2d> inside = pointsInside(points, p, radius);

            if (inside.size() > bestCount) {
                bestCenter = p;
                bestPoints = inside;
                bestCount = inside.size();
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {

                Translation2d a = points.get(i);
                Translation2d b = points.get(j);

                double d = a.getDistance(b);
                if (d > 2 * radius || d == 0) continue;

                double mx = (a.getX() + b.getX()) / 2.0;
                double my = (a.getY() + b.getY()) / 2.0;

                double h = Math.sqrt(radius * radius - (d / 2.0) * (d / 2.0));

                double dx = (b.getX() - a.getX()) / d;
                double dy = (b.getY() - a.getY()) / d;

                Translation2d center1 = new Translation2d(
                        mx - dy * h,
                        my + dx * h
                );

                Translation2d center2 = new Translation2d(
                        mx + dy * h,
                        my - dx * h
                );

                for (Translation2d center : Arrays.asList(center1, center2)) {
                    List<Translation2d> inside = pointsInside(points, center, radius);

                    if (inside.size() > bestCount) {
                        bestCenter = center;
                        bestPoints = inside;
                        bestCount = inside.size();
                    }
                }
            }
        }

        return new ClusterResult(bestCenter, bestPoints);
    }


    
}
