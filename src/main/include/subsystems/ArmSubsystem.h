// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
#pragma once

#include <frc2/command/SubsystemBase.h>

//Constants
#include "Constants.h"

//Driver Library
#include "rev/CanSparkMax.h"

//CanCoder Library
#include "ctre/Phoenix.h"

//PID
#include <frc/controller/PIDController.h>

#include <frc2/command/CommandPtr.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  double HightConePosition(bool left);

  double HightPosition(bool y);
  
  double MidPosition(bool b);

  double LowPosition(bool a);

  void SetArmPosition(bool Y, bool B, bool A);

  void DefaultPosition(); 

  double GetAbsolute();

  ctre::phoenix::sensors::CANCoder EncoderCan{35};


   * Will be called periodically whenever the CommandScheduler runs.

  void Periodic() override;
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax ArmMotor{50, rev::CANSparkMax::MotorType::kBrushless};


  frc::PIDController ArmPidController{
  ArmConstants::kParm, 0, 0};

      
  double DesiredPosition = ArmConstants::Default;
  // bool HightBoolean = false;
  // bool MidBoolean = false;
  // bool LowBoolean = false;

  bool Y = false;
  bool B = false;
  bool A = false;
};

*/

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

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // void  SpitIntake(double SpitSpeed_Parameter); 
  frc2::CommandPtr  upArmCommand(double ArmSpeed_Parameter);
  frc2::CommandPtr  downArmCommand(double ArmSpeed_Parameter);
  frc2::CommandPtr  StopArmCommand();
  frc2::CommandPtr  KeepArmCommand();
  // void StopIntake();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Crating the drivers Objects
  rev::CANSparkMax armMotor{14,rev::CANSparkMax::MotorType::kBrushless};

  //Implemating the 2 motors in just one object ->  GroupController class
  frc::MotorControllerGroup armIntake_Group{armMotor};

  //Speed of Catch
  double CatchSpeed;

    //Speed of Spit
  double SpitSpeed;

  
  frc::SlewRateLimiter<units::dimensionless::scalar> ArmFilter{0.1 / 20_ms};
};
