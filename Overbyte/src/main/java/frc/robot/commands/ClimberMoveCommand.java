// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMoveCommand extends CommandBase {
  ClimberSubsystem climber;

  /** Creates a new ClimberMoveCommand. */
  public ClimberMoveCommand(ClimberSubsystem climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    // This is checking if the robot is in teleop mode. If it is, it will move the climber based on the
    // axis values. If it is not, it will move the climber at 20% speed.
    if (RobotState.isTeleop()){
      climber.move(Constants.driveController.getRawAxis(Constants.extendLiftAxis) - Constants.driveController.getRawAxis(Constants.retractLiftAxis));
    }else{
      climber.moveLeft(-Constants.driveController.getRawAxis(Constants.extendLiftAxis) * 0.2);
      climber.moveRight(-Constants.driveController.getRawAxis(Constants.retractLiftAxis) * 0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
