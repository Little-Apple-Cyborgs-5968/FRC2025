// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.RoboSingSubsytem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RoboSingCommand extends Command {
  private final RoboSingSubsytem m_roboSing; 

  
  /** Creates a new RoboSingCommand. */
  public RoboSingCommand(RoboSingSubsytem roboSing) {
    m_roboSing = roboSing;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_roboSing);

    m_roboSing.playRobotRock();
    
  }

  // public void playRock(){
  //   m_roboSing.playRobotRock();
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
