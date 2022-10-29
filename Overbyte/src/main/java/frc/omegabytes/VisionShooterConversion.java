// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.omegabytes;

/** Add your docs here. */
public class VisionShooterConversion {
    private VisionConfiguration[] visionTable;
    private ShooterConfiguration[] shooterTable;
    private int hoodChangePosition = -1;

    public VisionShooterConversion(VisionConfiguration[] visionTable, ShooterConfiguration[] shooterTable, int hoodChangePosition) {
        this.visionTable = visionTable;
        this.shooterTable = shooterTable;

        // TODO Test this code to automatically determine the value of hoodChangePosition
//        for (int i = 1; i < shooterTable.length; i++) {
//            if (shooterTable[i].isHoodUp() != shooterTable[0].isHoodUp()) {
//                this.hoodChangePosition = i - 1;
//            }
//        }
        this.hoodChangePosition = hoodChangePosition - 1;
//        System.out.println("Hood change position is " + this.hoodChangePosition);
        assert(this.hoodChangePosition >= 0);
    }
    
    public ShooterConfiguration getValuesFromAngle(double angle, boolean isHoodUp){
        double distance = getDistanceFromAngle(angle);
        //System.out.println("DEBUG: Angle " + String.format("%.2f", angle) + " ==> distance " + String.format("%.2f", distance));
        int index = getDistanceIndex(distance, isHoodUp);
        double topMotor = getTopMotor(index, distance);
        double bottomMotor = getBottomMotor(index, distance);
        boolean setHoodUp = getHood(index);

        if (index == -1){
            //System.out.println("Robot cannot shoot from this position");
            topMotor = 0.0;
            bottomMotor = 0.0;
            setHoodUp = isHoodUp;
        }

        return new ShooterConfiguration(distance, topMotor, bottomMotor, setHoodUp);
    }

    public ShooterConfiguration getValuesFromDistance(double distance, boolean isHoodUp){
        int index = getDistanceIndex(distance, isHoodUp);
        double topMotor = getTopMotor(index, distance);
        double bottomMotor = getBottomMotor(index, distance);
        boolean setHoodUp = getHood(index);

        if (index == -1){
            //System.out.println("Robot cannot shoot from this position");
            topMotor = 0.0;
            bottomMotor = 0.0;
            setHoodUp = isHoodUp;
        }

        return new ShooterConfiguration(distance, topMotor, bottomMotor, setHoodUp);
    }

    /**
     * > This function returns the hood status of the shooter at the given index
     * 
     * @param index The index of the shooter table to get the hood value from.
     * @return A boolean value
     */
    private boolean getHood(int index){
        
        if (index < 1){
            index = 1;
        }

        if (index > shooterTable.length){
            index = shooterTable.length - 1;
        }
        
        return shooterTable[index].isHoodUp();
    }

    /**
     * It takes the index of the closest point in the shooter table, and the distance to the target,
     * and returns the speed of the bottom motor
     * 
     * @param index The index of the shooter table that is closest to the distance.
     * @param distance The distance to the target in inches
     * @return The speed of the bottom motor.
     */
    private double getBottomMotor(int index, double distance){
        // This is the code that determines the speed of the bottom motor. It is a linear interpolation
        // between the two closest points in the shooter table.
        double bottomMotor = 0.0;
        
        if (index < 1){
            index = 1;
        }

        if (index > shooterTable.length){
            index = shooterTable.length - 1;
        }

        double speedDifference = shooterTable[index].getBottomMotorSpeed() - shooterTable[index - 1].getBottomMotorSpeed();

        double distancePosition = distance - shooterTable[index - 1].getDistance();
        double distanceDifference = shooterTable[index].getDistance() - shooterTable[index - 1].getDistance();

        double distancePercentage = distancePosition / distanceDifference;
        bottomMotor = shooterTable[index - 1].getBottomMotorSpeed() + (speedDifference * distancePercentage);

        return bottomMotor;
    }

    private double getTopMotor(int index, double distance){
        double topMotor = 0.0;

        if (index < 1){
            index = 1;
        }

        if (index > shooterTable.length) {
            index = shooterTable.length - 1;
        }
        
        double speedDifference = shooterTable[index].getTopMotorSpeed() - shooterTable[index - 1].getTopMotorSpeed();

        double distancePosition = distance - shooterTable[index - 1].getDistance();
        double distanceDifference = shooterTable[index].getDistance() - shooterTable[index - 1].getDistance();

        double distancePercentage = distancePosition / distanceDifference;
        topMotor = shooterTable[index - 1].getTopMotorSpeed() + (speedDifference * distancePercentage);

        return topMotor;
    }
 
    // TODO Why not just return configuration directly?
    private int getDistanceIndex(double distance, boolean isHoodUp){
        int index = -1;
        boolean hoodUp;

        if (distance != 0.0){
            // Determine if the hood position will need to change, but prefer that it does not
            if (isHoodUp){
                if (distance < shooterTable[hoodChangePosition + 1].getDistance()){
                    hoodUp = false;
                }else{
                    hoodUp = true;
                }
            }else{
                if (distance > shooterTable[hoodChangePosition].getDistance()){
                    hoodUp = true;
                }else{
                    hoodUp = false;
                }
            }

            // Search through the appropriate configs (hood up or down) for the first entry that is beyond our shooting point
            if (hoodUp){
                for (int i = hoodChangePosition + 1; i < shooterTable.length; i++){
                    if (distance < shooterTable[i].getDistance()){
                        index = i;
                        break;
                    }
                }
            }else{
                for (int i = 1; i <= hoodChangePosition; i++){
                    if (distance < shooterTable[i].getDistance()){
                        index = i;
                        break;
                    }
                }
            }
        }

        //System.out.println("Shooting table index = " + index);
        return index;
    }

    /**
     * It takes an angle and returns the distance that the robot should be from the target at that
     * angle
     * 
     * @param angle The angle of the target from the limelight's perspective
     * @return The distance from the target.
     */
    private double getDistanceFromAngle(double angle){
        double distance = 0.0;
        for (int i = 1; i < visionTable.length; i++){
            if (visionTable[i].getAngle() < angle){
                double limelightAngleDifference = angle - visionTable[i].getAngle();
                double angleDifference = visionTable[i - 1].getAngle() - visionTable[i].getAngle();
                double distanceDifference = visionTable[i - 1].getDistance() - visionTable[i].getDistance();

                double anglePercentage = limelightAngleDifference / angleDifference;
                distance = visionTable[i].getDistance() + (distanceDifference * anglePercentage);

                break;
            }
        }

        //System.out.println(distance);
        return distance;
    }
}