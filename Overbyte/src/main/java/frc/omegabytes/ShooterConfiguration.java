// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class ShooterConfiguration {

    private double distance;
    private double topMotorSpeed;
    private double bottomMotorSpeed;
    private boolean hoodUp;

    public ShooterConfiguration(double distance, double topMotorSpeed, double bottomMotorSpeed, boolean hoodUp) {
        this.distance = distance;
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed; 
        this.hoodUp = hoodUp;
    }
 
    /**
     * This function returns the distance between the current location and the destination.
     * 
     * @return The distance between the two points.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * This function returns the speed of the top motor.
     * 
     * @return The top motor speed.
     */
    public double getTopMotorSpeed() {
        return topMotorSpeed;
    }

    /**
     * This function returns the speed of the bottom motor
     * 
     * @return The speed of the bottom motor.
     */
    public double getBottomMotorSpeed() {
        return bottomMotorSpeed;
    }

    public boolean isHoodUp() {
        return hoodUp;
    }
}
