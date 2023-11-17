// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.SwerveConstants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public TalonFX shooter_motor_bottomLeft;
  public TalonFX shooter_motor_bottomRight;

  private CurrentLimitsConfigs bottomLeftLimits;
  private CurrentLimitsConfigs bottomRightLimits;

  private TalonFXConfigurator bottomLeftConfigurator;
  private TalonFXConfigurator bottomRightConfigurator;

  private ClosedLoopRampsConfigs bottomLeftRampsConfigs;
  private ClosedLoopRampsConfigs bottomRightRampsConfigs;

  private Slot0Configs bottomLeft0Configs;
  private Slot0Configs bottomRight0Configs;

  public CANcoder canCoder;

  private MotorOutputConfigs motorConfigs;  

  public Shooter(int canCoderID) {
    canCoder = new CANcoder(canCoderID);

    motorConfigs = new MotorOutputConfigs();

    shooter_motor_bottomRight = new TalonFX(Constants.MotorConstants.FALCON_MOTOR1_CAN_CODER_ID);
    shooter_motor_bottomLeft = new TalonFX(Constants.MotorConstants.FALCON_MOTOR2_CAN_CODER_ID);
  
    bottomLeftConfigurator = shooter_motor_bottomLeft.getConfigurator();
    bottomRightConfigurator = shooter_motor_bottomRight.getConfigurator();

    bottomLeft0Configs = new Slot0Configs();
    bottomRight0Configs = new Slot0Configs();

    shooter_motor_bottomLeft.getConfigurator().apply(new TalonFXConfiguration());
    shooter_motor_bottomRight.getConfigurator().apply(new TalonFXConfiguration());

    // motorConfigs.NeutralMode = NeutralModeValue.Brake;
    // bottomLeftConfigurator.apply(motorConfigs);
    // bottomRightConfigurator.apply(motorConfigs);

    bottomLeft0Configs.kP = ShooterConstants.PIDConstants.FALCON_MOTOR1_PID.p;
    bottomLeft0Configs.kI = ShooterConstants.PIDConstants.FALCON_MOTOR1_PID.i;
    bottomLeft0Configs.kP = ShooterConstants.PIDConstants.FALCON_MOTOR1_PID.d;
    
    bottomRight0Configs.kP = ShooterConstants.PIDConstants.FALCON_MOTOR2_PID.p;
    bottomRight0Configs.kI = ShooterConstants.PIDConstants.FALCON_MOTOR2_PID.i;
    bottomRight0Configs.kD = ShooterConstants.PIDConstants.FALCON_MOTOR2_PID.d;
  
    shooter_motor_bottomLeft.getConfigurator().apply(bottomLeft0Configs);
    shooter_motor_bottomRight.getConfigurator().apply(bottomRight0Configs);
  }

  public void setSpeed(double speed){
    shooter_motor_bottomRight.set(speed);
    shooter_motor_bottomLeft.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}