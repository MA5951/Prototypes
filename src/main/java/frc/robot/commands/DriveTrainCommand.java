// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainCommand extends CommandBase {
  /** Creates a new DriveTrainCommand. */
  private DriveTrain driveTrain;
  private Joystick LJ;
  private Joystick RJ;
  public DriveTrainCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = DriveTrain.getInstance();
    LJ = new Joystick(0);
    RJ = new Joystick(1);
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.leftDrive(-LJ.getY());
    driveTrain.rightDrive(-RJ.getY());
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
