
package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class VelocityInterpolator {

    // A simple container for your calibration points
    private class Point {
        double distance;
        double velocity;

        public Point(double d, double v) {
            this.distance = d;
            this.velocity = v;
        }
    }

    // List to store unlimited calibration points
    private ArrayList<Point> frontTable = new ArrayList<>();
    private ArrayList<Point> backTable = new ArrayList<>();

    /**
     * Add a known measurement to the table.
     * Order doesn't matter; it will sort automatically.
     *
     * @param distance The measured distance from the target
     * @param launcherVelocity The velocity (RPM/Power) that worked at that distance
     */
    public void add(double distance, double launcherVelocity, double backVelocity) {
        frontTable.add(new Point(distance, launcherVelocity));
        backTable.add(new Point(distance, backVelocity));
        // Sort by distance (smallest to largest) every time we add a point
        Collections.sort(frontTable, Comparator.comparingDouble(p -> p.distance));
        Collections.sort(backTable, Comparator.comparingDouble(p -> p.distance));
    }

    /**
     * Calculates the target velocity for ANY distance.
     * - If between points: Interpolates (connects the dots).
     * - If outside range: Extrapolates (continues the line).
     */
    public double getVelocityForLauncher(double inputDistance) {
        int n = frontTable.size();

        // Safety: If no points, return 0
        if (n == 0) return 0.0;
        // If only one point, return that velocity always
        if (n == 1) return frontTable.get(0).velocity;

        // 1. Handle "Too Close" (Extrapolation below lowest point)
        if (inputDistance <= frontTable.get(0).distance) {
            return interpolate(inputDistance, frontTable.get(0), frontTable.get(1));
        }

        // 2. Handle "Too Far" (Extrapolation above highest point)
        if (inputDistance >= frontTable.get(n - 1).distance) {
            return interpolate(inputDistance, frontTable.get(n - 2), frontTable.get(n - 1));
        }

        // 3. Handle "In Between" (Standard Interpolation)
        for (int i = 0; i < n - 1; i++) {
            Point p1 = frontTable.get(i);
            Point p2 = frontTable.get(i + 1);

            // Check if our input falls between these two points
            if (inputDistance >= p1.distance && inputDistance < p2.distance) {
                return interpolate(inputDistance, p1, p2);
            }
        }

        return 0.0; // Should never reach here
    }

    // The math formula to find Y given X between two points
    private double interpolate(double x, Point p1, Point p2) {
        double slope = (p2.velocity - p1.velocity) / (p2.distance - p1.distance);
        return p1.velocity + slope * (x - p1.distance);
    }
    public double getVelocityForBack(double inputDistance) {
        int n = backTable.size();

        // Safety: If no points, return 0
        if (n == 0) return 0.0;
        // If only one point, return that velocity always
        if (n == 1) return backTable.get(0).velocity;

        // 1. Handle "Too Close" (Extrapolation below lowest point)
        if (inputDistance <= backTable.get(0).distance) {
            return interpolate(inputDistance, backTable.get(0), backTable.get(1));
        }

        // 2. Handle "Too Far" (Extrapolation above highest point)
        if (inputDistance >= backTable.get(n - 1).distance) {
            return interpolate(inputDistance, backTable.get(n - 2), backTable.get(n - 1));
        }

        // 3. Handle "In Between" (Standard Interpolation)
        for (int i = 0; i < n - 1; i++) {
            Point p1 = backTable.get(i);
            Point p2 = backTable.get(i + 1);

            // Check if our input falls between these two points
            if (inputDistance >= p1.distance && inputDistance < p2.distance) {
                return interpolate(inputDistance, p1, p2);
            }
        }

        return 0.0; // Should never reach here
    }
}