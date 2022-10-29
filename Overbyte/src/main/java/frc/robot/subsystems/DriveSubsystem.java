// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  // Creating a new SwerveDriveKinematics object.
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          new Translation2d(Constants.wheelBase / 2.0, Constants.wheelBase / 2.0),
          new Translation2d(Constants.wheelBase / 2.0, -Constants.wheelBase / 2.0),
          new Translation2d(-Constants.wheelBase / 2.0, Constants.wheelBase / 2.0),
          new Translation2d(-Constants.wheelBase / 2.0, -Constants.wheelBase / 2.0)
  );

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0.0)); 
  private Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  private Translation2d offsetPose = new Translation2d(0.0, 0.0);

  
  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.

  private ADIS16470_IMU gyro = new ADIS16470_IMU(IMUAxis.kZ, SPI.Port.kOnboardCS0, CalibrationTime._1s);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule flm;
  private final SwerveModule frm;
  private final SwerveModule rlm;
  private final SwerveModule rrm;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private boolean halfSpeed = false;
  private boolean robotOriented = false;

  public DriveSubsystem() {
    // Creating a new SwerveModule object.
    flm = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.fldmPort,
            Constants.flsmPort,
            Constants.flePort,
            Constants.fleo
    );

    frm = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.frdmPort,
            Constants.frsmPort,
            Constants.frePort,
            Constants.freo
    );

    rlm = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.rldmPort,
            Constants.rlsmPort,
            Constants.rlePort,
            Constants.rleo
    );

    rrm = Mk4SwerveModuleHelper.createFalcon500(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.rrdmPort,
            Constants.rrsmPort,
            Constants.rrePort,
            Constants.rreo
    );

    updateAngle();
  }

  /**
   * This function checks the angle of each motor and returns a boolean value
   */
  public void updateAngle() {
    SmartDashboard.putBoolean("FLM Angle OK", flm.checkAngle());
    SmartDashboard.putBoolean("FRM Angle OK", frm.checkAngle());
    SmartDashboard.putBoolean("RLM Angle OK", rlm.checkAngle());
    SmartDashboard.putBoolean("RRM Angle OK", rrm.checkAngle());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */

  public void zeroGyroscope() {
    gyro.calibrate();
  }

  /**
   * Returns the rotation of the robot in the field.
   * 
   * @return The rotation of the robot
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Returns the translation of the pose of the robot.
   * 
   * @return The translation of the pose of the robot.
   */
  public Translation2d getTranslation(){
    return getPose().getTranslation();
  }

  public Pose2d getPose(){
    return new Pose2d(robotPose.getTranslation().minus(offsetPose), robotPose.getRotation());
  }

  public SwerveDriveKinematics getKinematics(){
    return kinematics;
  }

  /**
   * This function sets the offset pose to a new pose
   * 
   * @param xValue The x value of the new pose
   * @param yValue The y value of the offset pose.
   */
  public void resetPose(double xValue, double yValue){
    offsetPose = new Translation2d(xValue, yValue);
  }

  /**
   * This function resets the robot's pose to the given x and y coordinates
   */
  public void resetPose(){
    resetPose(robotPose.getTranslation().getX(), robotPose.getTranslation().getY());
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void toggleHalfSpeed(){
    halfSpeed = !halfSpeed;
  }

  public boolean isHalfSpeed(){
    return halfSpeed;
  }

  /**
   * If the robot is oriented, it will be unoriented. If the robot is unoriented, it will be oriented
   */
  public void toggleRobotOriented(){
    robotOriented = !robotOriented;
  }

  /**
   * This function returns a boolean value that indicates whether the robot is oriented or not
   * 
   * @return The boolean value of robotOriented.
   */
  public boolean isRobotOriented(){
    return robotOriented;
  }

  /**
   * This function sets the chassis speeds to the given chassis speeds.
   * 
   * @param chassisSpeeds The desired speeds of the robot.
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  public void stop(){
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);

    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), states);
    robotPose = odometry.getPoseMeters();

    flm.set(0.0, Math.toRadians(0.0));
    frm.set(0.0, Math.toRadians(0.0));
    rlm.set(0.0, Math.toRadians(0.0));
    rrm.set(0.0, Math.toRadians(0.0));
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);

    flm.set(states[0].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[0].angle.getRadians());
    frm.set(-states[1].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[1].angle.getRadians());
    rlm.set(states[2].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[2].angle.getRadians());
    rrm.set(-states[3].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[3].angle.getRadians());

    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), states);
    robotPose = odometry.getPoseMeters();

    SmartDashboard.putNumber("Pose X", robotPose.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", robotPose.getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation", robotPose.getRotation().getDegrees());
}

  @Override
  public void periodic() {
    if (!RobotState.isAutonomous()){
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxVelocity);

      odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), states);

      
      robotPose = odometry.getPoseMeters();

      flm.set(states[0].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[0].angle.getRadians());
      frm.set(-states[1].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[1].angle.getRadians());
      rlm.set(states[2].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[2].angle.getRadians());
      rrm.set(-states[3].speedMetersPerSecond / Constants.maxVelocity * Constants.maxVoltage, states[3].angle.getRadians());
    }
      
    Pose2d poseData = getPose();

    SmartDashboard.putNumber("Pose X", poseData.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", poseData.getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation", poseData.getRotation().getDegrees());
  }
}
