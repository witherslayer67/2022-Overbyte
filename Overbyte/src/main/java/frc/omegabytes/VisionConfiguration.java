// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class VisionConfiguration {

    double angle;
    double distance;

    // This is a constructor. It is a special method that is called when an object is created from a
    // class and it allows the programmer to set the initial values for the member variables.
    public VisionConfiguration(double angle, double distance) {
        this.angle = angle;
        this.distance = distance;
    }
    
    /**
     * This function returns the angle of the object.
     * 
     * @return The angle of the object.
     */
    public double getAngle() {
        return angle;
    }

    /**
     * This function returns the distance between the current location and the destination.
     * 
     * @return The distance between the two points.
     */
    public double getDistance() {
        return distance;
    }
}
