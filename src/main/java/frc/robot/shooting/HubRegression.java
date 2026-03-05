package frc.robot.shooting;

public class HubRegression {
        public static double[][] kHoodManualAngle = {

                        // @x --> distance from target (in meters)
                        // @y --> hood angle (in degrees)
                        // { 1.2, 11.8 }, // hub
                        // { 1.7, 11.8 }, // corner hub
                        // { 2.15, 13.0},
                        // { 2.3, 13.0 },
                        // { 2.9, 14.5 }, // tower
                        // { 3.2, 15.0 },
                        // { 3.4, 15.5 }, // trench
                        // { 3.7, 17.0 }, // depot
                        // { 4.0, 18.0 },
                        // { 4.5, 19.0 },
                        // { 5.0, 21.5 },
                        // { 5.3, 22.0 } // corner
                        { 1.43, 11.8 }, // hub
                        { 1.7, 11.8 }, // corner hub
                        { 2.15, 13.0},
                        { 2.7, 13.0 },
                        { 3.1, 14.5 }, // tower
                        { 3.5, 15.5 }, // trench
                        { 3.85, 16.0 }, // depot
                        { 4.4, 17.0 },
                        { 5.0, 20.0 },
                        { 5.3, 22.0 } // corner
        };

        public static double[][] kFlywheelManualRPM = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> shooter velocity (in rpm)
                        // { 1.2, 1800}, // hub ***
                        // { 1.7, 1875}, // corner hub ***
                        // { 2.15, 1950},
                        // { 2.3, 2000}, // ***
                        // { 2.9, 2050}, // tower
                        // { 3.2, 2100},
                        // { 3.4, 2100}, // trench ***
                        // { 3.7, 2275}, // depot ***
                        // { 4.0, 2325}, // ***
                        // { 4.5, 2425}, // ***
                        // { 5.0, 2525}, // ***
                        // { 5.3, 2500} // corner ****
                        { 1.43, 1700}, // hub ***
                        { 1.85, 1750}, // corner hub ***
                        { 2.15, 1750},
                        { 2.7, 1850}, // ***
                        { 3.1, 1950}, // tower
                        { 3.5, 2000}, // trench ***
                        { 3.85, 2100}, // depot ***
                        { 4.4, 2200}, // ***
                        { 5.0, 2250}, // ***
                        { 5.3, 2300} // corner ****
        };

        public static double[][] kTimeOfFlightMap = {
                        // Need to change values

                        // @x --> distance from target (in meters)
                        // @y --> time of flight (in seconds)
                        { 1.61, 0.90 },
                        { 1.88, 1.09 },
                        { 3.15, 1.11 },
                        { 4.55, 1.12 },
                        { 5.68, 1.16 }
        };
}