// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimberMotor = new TalonFX(Constants.leftClimberMotorPort);
    rightClimberMotor = new TalonFX(Constants.rightClimberMotorPort);
  }

  /**
   * This function sets the speed of the left and right motors to the speed parameter, and then gets
   * the current of the left and right motors
   * 
   * @param speed the speed at which the climber will move.
   */
  public void move(double speed){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, speed * -1.0);
    leftClimberMotor.getStatorCurrent();
    rightClimberMotor.getStatorCurrent();
  }

  /**
   * This function sets the left climber motor to a certain speed.
   * 
   * @param speed The speed at which the motor will run. This is a double between -1.0 and 1.0.
   */
  public void moveLeft(double speed){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
  }

  /**
   * This function sets the right climber motor to a certain speed.
   * 
   * @param speed the speed at which the motor will run. This is a double between -1.0 and 1.0.
   */
  public void moveRight(double speed){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
  }

  /**
   * This function sets the left climber motor to a negative 15% output.
   */
  public void resetLeft(){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, -0.15);
  }  

  /**
   * This function sets the right climber motor to a speed of 0.15.
   */
  public void resetRight(){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, 0.15);
  }  

  /**
   * Stop the left climber motor.
   */
  public void stopLeft(){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  /**
   * Stop the right climber motor.
   */
  public void stopRight(){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void stop(){
    stopLeft();
    stopRight();
  }

  @Override
  public void periodic() {
  }
}