// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// conection terminated
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveManuallyCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final Timer locateTimer;

  private SlewRateLimiter translationXLimiter;
  private SlewRateLimiter translationYLimiter;
  private SlewRateLimiter rotationLimiter;

  private double translationXPercent;
  private double translationYPercent;
  private double rotationPercent;

  

    public DriveManuallyCommand(DriveSubsystem drive, IntakeSubsystem intake, VisionSubsystem vision){
        this.drive = drive;
        this.intake = intake;
        this.vision = vision;
        addRequirements(drive);

        locateTimer = new Timer();
        locateTimer.start();

        // Limiting the rate of change of the translationX, translationY, and rotation.
        translationXLimiter = new SlewRateLimiter(Constants.translationRateLimit);
        translationYLimiter = new SlewRateLimiter(Constants.translationRateLimit);
        rotationLimiter = new SlewRateLimiter(Constants.rotationRateLimit);
        
    }

    @Override
    public void execute() {
        // This is getting the values from the controller.
        translationXPercent = -Constants.driveController.getRawAxis(1);
        translationYPercent = -Constants.driveController.getRawAxis(0);
        rotationPercent = -Constants.driveController.getRawAxis(4);



        // This is a deadzone. It makes sure that the robot doesn't move when the joystick is in the
        // deadzone.
        if (Math.abs(translationXPercent) < Constants.deadzone){
            translationXPercent = 0.0;
        }
        
        // Making sure that the robot doesn't move when the joystick is in the deadzone.
        if (Math.abs(translationYPercent) < Constants.deadzone){
            translationYPercent = 0.0;
        }
        
        // Making sure that the robot doesn't move when the joystick is in the deadzone.
        if (Math.abs(rotationPercent) < Constants.deadzone){
            rotationPercent = 0.0;
        }

        // Multiplying the translationXPercent, translationYPercent, and rotationPercent by 1.00.
        translationXPercent *= 1.00;
        translationYPercent *= 1.00;
        rotationPercent *= 1.00;

        // This is a hack to make the robot stop looking for the target when the driver is driving.
        if (rotationPercent != 0.0 || (intake.isIntaking() && locateTimer.get() < 1.5)){
            locateTimer.reset();
        }

       
        // This is a hack to make the robot stop looking for the target when the driver is driving.
        if ((Constants.driveController.getRawButton(Constants.readyToShootButton))) { 
            double x = vision.getPosition();
            if (Math.abs(x) >= Constants.visionAnglePrecision) {
                rotationPercent = .1 * -Math.signum(x);
            }
        }

        translationXPercent = translationXLimiter.calculate(translationXPercent);
        translationYPercent = translationYLimiter.calculate(translationYPercent);
        rotationPercent = rotationLimiter.calculate(rotationPercent);

        // This is a deadzone. It makes sure that the robot doesn't move when the joystick is in the
        //         // deadzone.
        if (Math.abs(translationXPercent) < Constants.deadzone){
            translationXPercent = 0.0;
        }
        
        // Making sure that the robot doesn't move when the joystick is in the deadzone.
        if (Math.abs(translationYPercent) < Constants.deadzone){
            translationYPercent = 0.0;
        }
        
        if (Math.abs(rotationPercent) < Constants.deadzone){
            rotationPercent = 0.0;
        }
         
        // Driving the robot.
        drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXPercent * Constants.maxVelocity * (drive.isHalfSpeed() ? 0.5 : 1.0), 
                translationYPercent * Constants.maxVelocity * (drive.isHalfSpeed() ? 0.5 : 1.0),
                rotationPercent * Constants.maxAngularVelocity * (drive.isHalfSpeed() ? Constants.driveController.getRawButton(Constants.readyToShootButton) ? 1.0 : 0.5 : 1.0),
                (drive.isRobotOriented() ? Rotation2d.fromDegrees(0.0) : drive.getGyroscopeRotation())
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
