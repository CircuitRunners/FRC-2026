package frc.robot.shooting;

public class PassingRegression {
        public static double[][] kPassingHoodManualAngle = {

                        // @x --> distance from target (in meters)
                        // @y --> hood angle (in degrees)

                        { 5.46, 33.0 },
                        { 6.62, 33.0 },
                        { 7.8,  33.0 },
                        { 17.16, 33.0}
        };

        public static double[][] kPassingFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        { 5.46, 2000.0 },
                        { 6.62, 2200.0 },
                        { 7.8,  2400.0 },
                        { 17.16,3400.0 }
        };

        public static double[][] kPassingTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 5.46, 1.27 },
                        { 6.62, 1.39 },
                        { 7.8 , 1.49 },
                        { 11.0, 1.75 },
                        { 13.0, 1.76 },
                        { 17.16,2.16 }
        };
    }