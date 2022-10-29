// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class VisionWeightedAverage {
    private int valueCount;
    private double[] xValues;
    private double[] yValues;
    private double[] weights;

    
    private double xOut;
    private double yOut;
    
    // This is a constructor that takes in no parameters. It calls the other constructor with the
    // parameters 10 and an array of doubles.
    public VisionWeightedAverage() {
        this(10, new double[]{25.0, 20.0, 15.0, 10.0, 7.5, 7.5, 5.0, 5.0, 2.5, 2.5});
    }

    // A constructor that takes in two parameters.
    public VisionWeightedAverage(int valueCount, double[] weights) {
        this.valueCount = valueCount;
        this.weights = weights;
        clearValues();
    }

    public void clearValues(){
        xValues = new double[valueCount];
        yValues = new double[valueCount];
        for (int i = 0; i < valueCount; i++){
            xValues[i] = 0.0;
            yValues[i] = 0.0;
        }
        xOut = 0.0;
        yOut = 0.0;
    }



    /**
     * The update function takes in the current position and angle of the robot and stores them in the
     * first index of the xValues and yValues arrays. Then, it shifts all the values in the arrays down
     * one index, and calculates the output of the filter.
     * 
     * @param position The position of the robot
     * @param angle The angle of the robot in radians
     */
    public void update(double position, double angle){
        for (int i = valueCount - 1; i > 0; i--){
            xValues[i] = xValues[i - 1];
            yValues[i] = yValues[i - 1];
        }

        xValues[0] = position;
        yValues[0] = angle;

        xOut = calculateX();
        yOut = calculateY();
    }

    /**
     * This function returns the value of the first element in the array xValues
     * 
     * @return The value of the x-axis.
     */
    public double getX(){
        //return xOut;
        return xValues[0];
    }

    /**
     * This function returns the y-value of the first element in the yValues array.
     * 
     * @return The y-value of the first element in the array.
     */
    public double getY(){
        // TODO Temporarily disabled for testing
        //return yOut;
        return yValues[0];
    }

    /**
     * > Calculate the weighted average of the x values
     * 
     * @return The weighted average of the x values.
     */
    private double calculateX(){
        double value = 0.0;
        double weight = 0.0;

        for (int i = 0; i < valueCount; i++){
            if (xValues[i] != 0.0){
                value += xValues[i] * weights[i];
                weight += weights[i];
            }
        }

        return value / weight;
    }

    /**
     * "Calculate the weighted average of the y values."
     * 
     * The function is a little more complicated than that, but that's the gist of it
     * 
     * @return The weighted average of the y values.
     */
    private double calculateY(){
        double value = 0.0;
        double weight = 0.0;

        for (int i = 0; i < valueCount; i++){
            if (yValues[i] != 0.0){
                value += yValues[i] * weights[i];
                weight += weights[i];
            }
        }

        return value / weight;
    }

}
