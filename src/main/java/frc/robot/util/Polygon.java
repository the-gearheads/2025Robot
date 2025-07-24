package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Polygon {
    private final int numPoints;
    private final double[] xCoordinates;
    private final double[] yCoordinates;

    public Polygon(double[] xCoordinates, double[] yCoordinates) {
        if (xCoordinates.length != yCoordinates.length || xCoordinates.length < 3) {
            throw new IllegalArgumentException("Invalid number of coordinates for a polygon");
        }
        this.numPoints = xCoordinates.length;
        this.xCoordinates = xCoordinates;
        this.yCoordinates = yCoordinates;
    }

    public boolean contains(Pose2d pose) {
        return contains(pose.getX(), pose.getY());
    }

    public boolean contains(Translation2d translation) {
        return contains(translation.getX(), translation.getY());
    }

    public boolean contains(double x, double y) {
        int crossings = 0;
        for (int i = 0; i < numPoints; i++) {
            int next = (i + 1) % numPoints;
            double x1 = xCoordinates[i];
            double y1 = yCoordinates[i];
            double x2 = xCoordinates[next];
            double y2 = yCoordinates[next];

            if (((y1 <= y && y < y2) || (y2 <= y && y < y1)) &&
                    (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1)) {
                crossings++;
            }
        }
        return crossings % 2 != 0;
    }
}