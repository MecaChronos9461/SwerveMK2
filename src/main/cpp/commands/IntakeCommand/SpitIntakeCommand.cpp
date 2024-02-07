// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/SpitIntakeCommand.h"

SpitIntakeCommand::SpitIntakeCommand(IntakeSubsystem*intake_SubsystemParameter):intake_Subsystem(intake_SubsystemParameter)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake_SubsystemParameter);

}

// Called when the command is initially scheduled.
void SpitIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SpitIntakeCommand::Execute() {

  
  // intake_Subsystem->SpitIntake(-0.1);
}

// Called once the command ends or is interrupted.
void SpitIntakeCommand::End(bool interrupted) {


}

// Returns true when the command should end.
bool SpitIntakeCommand::IsFinished() {
  return false;
}
