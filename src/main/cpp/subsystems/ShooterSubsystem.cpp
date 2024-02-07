// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "iostream"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

ShooterSubsystem::ShooterSubsystem()

{
shooterLeftMotor.SetSmartCurrentLimit(0,35);
shooterLeftMotor.BurnFlash();
shooterLeftMotor.SetInverted(false);

shooterRightMotor.SetSmartCurrentLimit(0,35);
shooterRightMotor.BurnFlash();

shooterRightMotor.SetInverted(true);

}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
}


frc2::CommandPtr ShooterSubsystem::ShootAmpCommand(double shootAmplifier_speed){
   AmplifierSpeed = shootAmplifier_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { motorShooterGroup.Set(ShooterFilter.Calculate(AmplifierSpeed));});        
}
frc2::CommandPtr ShooterSubsystem::ShootSpeakerCommand(double shootSpeaker_speed){
   SpeakerSpeed = shootSpeaker_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { motorShooterGroup.Set(SpeakerSpeed);});        
}

frc2::CommandPtr ShooterSubsystem::StopShooterCommand(){
return this->RunOnce( 
      [this] { motorShooterGroup.StopMotor();});
}

frc2::CommandPtr ShooterSubsystem::WaitShooter3sCommand(){
return this->RunOnce( 
      [this] { frc2::WaitCommand(10.0_s);});
}

/*
frc2::CommandPtr ShooterSubsystem::ShootAmplifierCommand(double shootAmp_speed){
   AmplifierSpeed = shootAmp_speed;
   return this->RunOnce( 
      [this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
}*/

/*

frc2::CommandPtr IntakeSubsystem::SpitIntakeCommand(double spit_Speed){
   SpitSpeed = spit_Speed;
   return this->RunOnce( 
      [this] { motorIntake_Group.Set(IntakeFilter.Calculate(SpitSpeed));});

}

frc2::CommandPtr IntakeSubsystem::KeepIntakeCommand(){

    return this->RunOnce(
      [this] { motorIntake_Group.Set(-0.05); });
}

frc2::CommandPtr IntakeSubsystem::StopIntakeCommand(){

   return this->RunOnce(
      [this] {motorIntake_Group.Set(0.0);});

}
*/