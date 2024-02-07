// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//Driver Library
#include "rev/CanSparkMax.h"

//MotorGroup Library
#include "frc/motorcontrol/MotorControllerGroup.h"

#include "frc/filter/SlewRateLimiter.h"

#include "frc/filter/MedianFilter.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // void  SpitIntake(double SpitSpeed_Parameter); 
  frc2::CommandPtr  ShootSpeakerCommand(double SpeakerSpeed_Parameter);
  frc2::CommandPtr  ShootAmpCommand(double AmpSpeed_Parameter);
  frc2::CommandPtr  WaitShooter3sCommand();
  frc2::CommandPtr  StopShooterCommand();
  frc2::CommandPtr  KeepShooterCommand();
  // void StopIntake();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Crating the drivers Objects
  
  rev::CANSparkMax shooterLeftMotor{11,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax shooterRightMotor{12,rev::CANSparkMax::MotorType::kBrushless};

    

  //Implemating the 2 motors in just one object ->  GroupController class
  frc::MotorControllerGroup motorShooterGroup{shooterLeftMotor, shooterRightMotor};

  //Speed for Speaker
  double SpeakerSpeed;
  //Speed for Speaker
  double AmplifierSpeed;
  double ShooterWaitTime;

    //Speed for Amplifier
  double AmpSpeed;

bool ShooterState;
  
  frc::SlewRateLimiter<units::dimensionless::scalar> ShooterFilter{0.1 / 20_ms};
};
