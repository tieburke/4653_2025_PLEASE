package frc.lib.math;

public class Conversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts Neo Position Counts
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double neoToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism
     * @return Neo Position Counts
     */
    public static double degreesToNeo(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Neo Velocity Counts
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return RPM of Mechanism
     */
    public static double neoToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }


    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToNeo(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Neo Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Neo Velocity Counts
     */
    public static double neoToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = neoToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Mechanism (set to 1 for Neo MPS)
     * @return Neo Velocity Counts
     */
    public static double MPSToNeo(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToNeo(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Neo Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Meters
     */
    public static double neoToMeters(double positionCounts, double circumference, double gearRatio, int motorNumber){
        return (positionCounts * (circumference / (4096.0)))/ gearRatio;
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Neo and Wheel
     * @return Neo Position Counts
     */
    public static double MetersToNeo(double meters, double circumference, double gearRatio){
        return meters / (circumference / (4096.0 / gearRatio)) / gearRatio;
    }

    /**
     * @param positionCounts Neck Encoder Position Counts
     * @param gearRatio Gear Ratio between Neck Motor and Neck
     * @return Degrees of Rotation of Neck
     */
    public static double NeckToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

}