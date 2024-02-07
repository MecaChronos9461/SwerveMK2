// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberCommand.h"

ClimberCommand::ClimberCommand(ClimberSubsystem*climber_SubsystemParameter):climber_Subsystem(climber_SubsystemParameter)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber_SubsystemParameter);

}

// Called when the command is initially scheduled.
void ClimberCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberCommand::Execute() {

  
  // intake_Subsystem->SpitIntake(-0.1);
}

// Called once the command ends or is interrupted.
void ClimberCommand::End(bool interrupted) {


}

// Returns true when the command should end.
bool ClimberCommand::IsFinished() {
  return false;
}
