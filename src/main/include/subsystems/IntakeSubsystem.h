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

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // void  SpitIntake(double SpitSpeed_Parameter); 
  frc2::CommandPtr  SpitIntakeCommand(double SpitSpeed_Parameter);
  frc2::CommandPtr  CatchIntakeCommand(double CatchSpeed_Parameter);
  frc2::CommandPtr  StopIntakeCommand();
  frc2::CommandPtr  KeepIntakeCommand();
  // void StopIntake();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Crating the drivers Objects
  rev::CANSparkMax intakeMotor{13,rev::CANSparkMax::MotorType::kBrushless};

  //Implemating the 2 motors in just one object ->  GroupController class
  frc::MotorControllerGroup motorIntake_Group{intakeMotor};

  //Speed of Catch
  double CatchSpeed;

    //Speed of Spit
  double SpitSpeed;

  
  frc::SlewRateLimiter<units::dimensionless::scalar> IntakeFilter{0.1 / 20_ms};
};
