package frc.robot.shooting;

public class HubRegression {
        public static double[][] kHoodManualAngle = {

                        // @x --> distance from target (in meters)
                        // @y --> hood angle (in degrees)
                        { 1.2, 11.8 }, // hub
                        { 1.7, 11.8 }, // corner hub
                        { 2.3, 11.8 },
                        { 2.9, 14.5 }, // tower
                        { 3.4, 16.0 }, // trench
                        { 3.7, 17.0 }, // depot
                        { 4.0, 18.0 },
                        { 4.5, 19.0 },
                        { 5.0, 21.5 },
                        { 5.3, 22.5 } // corner
        };

        public static double[][] kFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        { 1.2, 1600}, // hub ***
                        { 1.7, 1700}, // corner hub ***
                        { 2.3, 1850}, // ***
                        { 2.9, 1900}, // tower
                        { 3.4, 2100}, // trench ***
                        { 3.7, 2200}, // depot ***
                        { 4.0, 2250}, // ***
                        { 4.5, 2400}, // ***
                        { 5.0, 2450}, // ***
                        { 5.3, 2450} // corner ****
        };

        public static double[][] kTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 1.38, 0.90 },
                        { 1.88, 1.09 },
                        { 3.15, 1.11 },
                        { 4.55, 1.12 },
                        { 5.68, 1.16 }
        };
}