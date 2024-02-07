// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include<iostream>

IntakeSubsystem::IntakeSubsystem()

{
intakeMotor.SetSmartCurrentLimit(0,35);
intakeMotor.BurnFlash();
intakeMotor.SetInverted(false);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
}


frc2::CommandPtr IntakeSubsystem::CatchIntakeCommand(double catch_Speed){
   CatchSpeed = catch_Speed;
   return this->RunOnce( 
      [this] { motorIntake_Group.Set(IntakeFilter.Calculate(CatchSpeed));});
      //[this] { motorIntake_Group.Set(0.2);});

}



frc2::CommandPtr IntakeSubsystem::SpitIntakeCommand(double spit_Speed){
   SpitSpeed = spit_Speed;
   return this->RunOnce( 
      [this] { motorIntake_Group.Set(-IntakeFilter.Calculate(SpitSpeed));});
      //[this] { motorIntake_Group.Set(0.2);});

}

frc2::CommandPtr IntakeSubsystem::KeepIntakeCommand(){

    return this->RunOnce(
      [this] { motorIntake_Group.Set(0.05); });
}

frc2::CommandPtr IntakeSubsystem::StopIntakeCommand(){

   return this->RunOnce(
      [this] {motorIntake_Group.Set(0.0);});

}

