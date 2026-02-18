package frc.robot.shooting;

public class PassingRegression {
        public static double[][] kPassingHoodManualAngle = {

                        // @x --> distance from target (in meters)
                        // @y --> hood angle (in degrees)

                        { 0.0, 0.0 },
                        { 100000.0, 0.0 }
        };

        public static double[][] kPassingFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        { 0.0, 0.0 },
                        { 100000.0, 0.0 }
        };

        public static double[][] kPassingTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 0.0, 0.0 },
                        { 100000.0, 0.0 }
        };
    }