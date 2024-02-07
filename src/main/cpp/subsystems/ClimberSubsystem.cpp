// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem()

{
climberLeftMotor.SetSmartCurrentLimit(0,35);
climberLeftMotor.BurnFlash();
climberLeftMotor.SetInverted(false);

climberRightMotor.SetSmartCurrentLimit(0,35);
climberRightMotor.BurnFlash();
climberRightMotor.SetInverted(false);
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {
}

//========= **CLIMBER** ==== *LEFT_MOTOR*=====
frc2::CommandPtr ClimberSubsystem::UpLeftClimberCommand(double upLeftClimber_speed){
   UpLeftClimberSpeed = upLeftClimber_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { climberLeftMotor.Set(-0.5);});        
}
frc2::CommandPtr ClimberSubsystem::DownLeftClimberCommand(double downLeftClimber_speed){
   DownLeftClimberSpeed = downLeftClimber_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { climberLeftMotor.Set(0.5);});        
}
frc2::CommandPtr ClimberSubsystem::StopLeftClimberCommand(){
return this->RunOnce( 
      [this] { climberLeftMotor.StopMotor();});
}

//========= **CLIMBER** ==== *RIGHT_MOTOR*=====
frc2::CommandPtr ClimberSubsystem::UpRightClimberCommand(double upLeftClimber_speed){
   UpRightClimberSpeed = upLeftClimber_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { climberRightMotor.Set(-0.5);});        
}
frc2::CommandPtr ClimberSubsystem::DownRightClimberCommand(double rightLeftClimber_speed){
   DownRightClimberSpeed = rightLeftClimber_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { climberRightMotor.Set(0.5);});        
}
frc2::CommandPtr ClimberSubsystem::StopRightClimberCommand(){
return this->RunOnce( 
      [this] { climberRightMotor.StopMotor();});
}

//========= **CLIMBER** ==== *MOTOR_GROUP*=====
frc2::CommandPtr ClimberSubsystem::UpClimberMotorGroupCommand(double upMotorGroupClimber_speed){
   UpMotorGroupClimberSpeed = upMotorGroupClimber_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { climberMotorGroup.Set(0.3);});        
}
frc2::CommandPtr ClimberSubsystem::DownClimberMotorGroupCommand(double downMotorGroupClimber_speed){
   DownMotorGroupClimberSpeed = downMotorGroupClimber_speed;
   return this->RunOnce( 
      //[this] { motorShooterGroup.Set(ShooterFilter.Calculate(SpeakerSpeed));});        
      [this] { climberMotorGroup.Set(-0.3);});        
}
frc2::CommandPtr ClimberSubsystem::StopClimberMotorGroupCommand(){
return this->RunOnce( 
      [this] { climberMotorGroup.StopMotor();});
}