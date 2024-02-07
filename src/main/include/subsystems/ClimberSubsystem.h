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

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //====Climber LEFT Motor=====
  frc2::CommandPtr  UpLeftClimberCommand(double upLeftClimberSpeed_Parameter);
  frc2::CommandPtr  DownLeftClimberCommand(double downLeftClimberSpeed_Parameter);
  frc2::CommandPtr  StopLeftClimberCommand();
//====Climber RIGHT Motor=====
  frc2::CommandPtr  UpRightClimberCommand(double upRightClimberSpeed_Parameter);
  frc2::CommandPtr  DownRightClimberCommand(double downRightClimberSpeed_Parameter);
  frc2::CommandPtr  StopRightClimberCommand();
//=====Climber Motor GORUP====
  frc2::CommandPtr  UpClimberMotorGroupCommand(double downMotorGroupClimberSpeed_Parameter);
  frc2::CommandPtr  DownClimberMotorGroupCommand(double downMotorGroupClimberSpeed_Parameter);
  frc2::CommandPtr  StopClimberMotorGroupCommand();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Crating the drivers Objects
  
  rev::CANSparkMax climberLeftMotor{15,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax climberRightMotor{16,rev::CANSparkMax::MotorType::kBrushless};

    

  //Implemating the 2 motors in just one object ->  GroupController class
  frc::MotorControllerGroup climberMotorGroup{climberLeftMotor, climberRightMotor};


  double UpLeftClimberSpeed;
  double DownLeftClimberSpeed;
  double UpRightClimberSpeed;
  double DownRightClimberSpeed;
  double UpMotorGroupClimberSpeed;
  double DownMotorGroupClimberSpeed;
 
  frc::SlewRateLimiter<units::dimensionless::scalar> ClimberFilter{0.1 / 20_ms};
};
