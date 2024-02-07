// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/IntakeCatchCommand.h"

IntakeCatchCommand::IntakeCatchCommand(IntakeSubsystem*intake_SubsystemParameter): intake_Subsystem(intake_SubsystemParameter) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake_SubsystemParameter);
}

// Called when the command is initially scheduled.
void IntakeCatchCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeCatchCommand::Execute() {

  // intake_Subsystem->CatchIntake();
  
}

// Called once the command ends or is interrupted.
void IntakeCatchCommand::End(bool interrupted) {

  
}

// Returns true when the command should end.
bool IntakeCatchCommand::IsFinished() {
  return false;
}
