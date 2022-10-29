// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  

  private boolean intaking = false;
  private TalonFX intakeMotor;
  private DoubleSolenoid intakeSolenoids;
  private DigitalInput beamBreakSensor;

  // These values are to counter the intake sensors picking each other up
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(DoubleSolenoid intakeSolenoids) {
    intakeMotor = new TalonFX(Constants.intakeMotorPort);
    this.intakeSolenoids = intakeSolenoids;
    
    beamBreakSensor = new DigitalInput(Constants.intakeBeamBreakSensorPort);

    retract();
    stopIntake();
  }

  /**
   * This function returns the value of the beam break sensor
   * 
   * @return The value of the beam break sensor.
   */
  public boolean getBeamBreakSensor(){
    return !beamBreakSensor.get(); // Inverting the value as the sensor returns true when no ball is in the way 
  }

  /**
   * > This function returns true if the intake is extended, and false if it is not
   * 
   * @return The intakeSolenoids.get() is being returned.
   */
  public boolean isExtended(){
    return (intakeSolenoids.get() == Value.kForward) ? true : false;
  }

  public void runIntake(){
    setIntakeSpeed(-0.75);
    intaking = true;
  }

  /**
   * This function sets the intake speed to 1.0, which is the reverse direction
   */
  public void reverseIntake(){
    setIntakeSpeed(1.0);
  }

  /**
   * This function sets the intake speed to 0.0 and sets the intaking boolean to false
   */
  public void stopIntake(){
    setIntakeSpeed(0.0);
    intaking = false;
  }

  /**
   * This function sets the speed of the intake motor to the speedPercent parameter
   * 
   * @param speedPercent The speed of the motor, from -1 to 1.
   */
  public void setIntakeSpeed(double speedPercent){
    intakeMotor.set(TalonFXControlMode.PercentOutput, speedPercent);
  }

  /**
   * This function sets the intake solenoids to extend the intake.
   */
  public void extend(){
    intakeSolenoids.set(Value.kForward);
  }

  /**
   * This function sets the intake solenoids to reverse, which retracts the intake
   */
  public void retract(){
    intakeSolenoids.set(Value.kReverse);
  }

  public void stop(){
    retract();
    stopIntake();
  }

  /**
   * This function returns a boolean value that is true if the robot is intaking and false if it is not
   * 
   * @return The boolean value of intaking.
   */
  public boolean isIntaking(){
    return intaking;
  }


  @Override
  public void periodic() {
    ////System.out.println("Intake: " + (getbeamBreakSensor() ? "Has " : "No ") + "Ball");
  }
}
