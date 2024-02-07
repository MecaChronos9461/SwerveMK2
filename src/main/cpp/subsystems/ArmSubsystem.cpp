// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

/*
ArmSubsystem::ArmSubsystem(){
    ArmPidController.EnableContinuousInput(0, 360);
    EncoderCan.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);
    ArmMotor.SetSmartCurrentLimit(0, 35);
    ArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    ArmMotor.BurnFlash();
    ArmPidController.Reset();
}

double ArmSubsystem::MidPosition(bool mid){
    
    if (mid){
      DesiredPosition = ArmConstants::Mid;}
    return DesiredPosition;
}
double ArmSubsystem::HightPosition(bool Hight){
    
    if(Hight){
        DesiredPosition = ArmConstants::Hight;}
    return DesiredPosition;
}
double ArmSubsystem::LowPosition(bool low){
   if(low){
     DesiredPosition = ArmConstants::Low;}
    return DesiredPosition;
}
void ArmSubsystem::DefaultPosition(){
    // if((!HightBoolean) && (!MidBoolean) && (!LowBoolean)){
    //  DesiredPosition = ArmConstants::Default;
    //  return DesiredPosition;
    // }
    
DesiredPosition = ArmConstants::Default;
}

void ArmSubsystem::SetArmPosition(bool y, bool b, bool a){
    A = a;
    B = b;
    Y = y;

    MidPosition(B);
    LowPosition(A);
    HightPosition(Y);

}

double ArmSubsystem::GetAbsolute(){


    return EncoderCan.GetAbsolutePosition();
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("ArmAbsolutePosition", EncoderCan.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("pid", ArmPidController.Calculate(EncoderCan.GetAbsolutePosition(), DesiredPosition));
    frc::SmartDashboard::PutNumber("error", ArmPidController.GetPositionError());
    frc::SmartDashboard::PutNumber("desired", DesiredPosition);
    frc::SmartDashboard::PutBoolean("A", A);
    frc::SmartDashboard::PutBoolean("B", B);
    frc::SmartDashboard::PutBoolean("Y", Y);
    ArmMotor.Set(ArmPidController.Calculate(EncoderCan.GetAbsolutePosition(), DesiredPosition));
}
*/


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include<iostream>

ArmSubsystem::ArmSubsystem()

{
armMotor.SetSmartCurrentLimit(0,35);
armMotor.BurnFlash();
armMotor.SetInverted(false);
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
}


frc2::CommandPtr ArmSubsystem::upArmCommand(double catch_Speed){
   CatchSpeed = catch_Speed;
   return this->RunOnce( 
      [this] { armIntake_Group.Set(0.2);});

}



frc2::CommandPtr ArmSubsystem::downArmCommand(double spit_Speed){
   SpitSpeed = spit_Speed;
   return this->RunOnce( 
      [this] { armIntake_Group.Set(-0.20);});

}

frc2::CommandPtr ArmSubsystem::KeepArmCommand(){

    return this->RunOnce(
      [this] { armIntake_Group.Set(-0.05); });
}

frc2::CommandPtr ArmSubsystem::StopArmCommand(){

   return this->RunOnce(
      [this] {armIntake_Group.StopMotor();});

}