// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommand.h"

ShooterCommand::ShooterCommand(ShooterSubsystem*shooter_SubsystemParameter):shooter_Subsystem(shooter_SubsystemParameter)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter_SubsystemParameter);

}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {

  
  // intake_Subsystem->SpitIntake(-0.1);
}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {


}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  return false;
}
