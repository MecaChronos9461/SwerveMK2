// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ControlSubsystem.h"

ControlSubsystem::ControlSubsystem() = default;

// This method will be called once per scheduler run
void ControlSubsystem::Periodic() {frc::SmartDashboard::PutBoolean("FieldOriented", GetFieldOriented());}

double ControlSubsystem::GetDriveX(){ return -joy1.GetRawAxis(1); }

double ControlSubsystem::GetDriveY(){ return joy1.GetRawAxis(0); }

double ControlSubsystem::GetRotX(){ return -joy1.GetRawAxis(4); }

bool ControlSubsystem::GetFieldOriented(){ 
    
    if(joy1.GetRawButton(5)){
        kField = false;
    }
    else if(joy1.GetRawButton(6)){
        kField = true;
    }
    return kField;
}