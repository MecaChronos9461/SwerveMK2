// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand/ArmCommand.h"

ArmCommand::ArmCommand(ArmSubsystem*arm_SubsystemParamter):arm_Subsystem{arm_SubsystemParamter} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm_SubsystemParamter);
}

// Called when the command is initially scheduled.
void ArmCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ArmCommand::Execute() {}

// Called once the command ends or is interrupted.
void ArmCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmCommand::IsFinished() {
  return false;
}
