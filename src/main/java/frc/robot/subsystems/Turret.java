// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public TalonFX leftturret_motor = new TalonFX(53);
  public TalonFX rightturret_motor = new TalonFX(54);

  public final CANcoder lcan1 = new CANcoder(3);
  public final CANcoder lcan2 = new CANcoder(5);
  public final CANcoder rcan1 = new CANcoder(6);
  public final CANcoder rcan2 = new CANcoder(4);

  public final MotionMagicDutyCycle m_TurretmagicDutyCycle;
  public final PositionDutyCycle m_TurretPosDutyCycle;
  public final TorqueCurrentConfigs m_TurretCurrentConfigs = new TorqueCurrentConfigs();

  /** Creates a new hood_subsystem. */
  public Turret() {
    CANcoderConfiguration lcan1_config = new CANcoderConfiguration();
    CANcoderConfiguration lcan2_config = new CANcoderConfiguration();
    CANcoderConfiguration rcan1_config = new CANcoderConfiguration();
    CANcoderConfiguration rcan2_config = new CANcoderConfiguration();

    lcan1_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    lcan2_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1).withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    rcan1_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    rcan2_config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1).withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    lcan1.getConfigurator().apply(lcan1_config, 0.050);
    lcan2.getConfigurator().apply(lcan2_config, 0.050);
    rcan1.getConfigurator().apply(rcan1_config, 0.050);
    rcan2.getConfigurator().apply(rcan2_config, 0.050);

    TalonFXConfiguration LeftTurretTalonConfigs = new TalonFXConfiguration();
    LeftTurretTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.6);
    LeftTurretTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.6);
    LeftTurretTalonConfigs.Slot0.withKP(0.1);
    LeftTurretTalonConfigs.Slot0.withKG(0.0);
    LeftTurretTalonConfigs.Slot0.withKI(0);
    LeftTurretTalonConfigs.Slot0.withKD(0);
    LeftTurretTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    LeftTurretTalonConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    LeftTurretTalonConfigs.MotionMagic.withMotionMagicAcceleration(150*1.5);
    LeftTurretTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(200*1.5);

    TalonFXConfiguration RightTurretTalonConfigs = new TalonFXConfiguration();
    RightTurretTalonConfigs.MotorOutput.withPeakForwardDutyCycle(0.6);
    RightTurretTalonConfigs.MotorOutput.withPeakReverseDutyCycle(-0.6);
    RightTurretTalonConfigs.Slot0.withKP(0.1);
    RightTurretTalonConfigs.Slot0.withKG(0.0);
    RightTurretTalonConfigs.Slot0.withKI(0);
    RightTurretTalonConfigs.Slot0.withKD(0);
    RightTurretTalonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    RightTurretTalonConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    RightTurretTalonConfigs.MotionMagic.withMotionMagicAcceleration(150*1.5);
    RightTurretTalonConfigs.MotionMagic.withMotionMagicCruiseVelocity(200*1.5);

    m_TurretmagicDutyCycle = new MotionMagicDutyCycle(0);
    m_TurretPosDutyCycle = new PositionDutyCycle(0);

    leftturret_motor.getConfigurator().apply(LeftTurretTalonConfigs, 0.050);
    rightturret_motor.getConfigurator().apply(RightTurretTalonConfigs, 0.050);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Turret Teeth rotated", convert_teeth_to_degrees(find_teeth_rotated(lcan1.getAbsolutePosition().getValueAsDouble(), lcan2.getAbsolutePosition().getValueAsDouble())));
    SmartDashboard.putNumber("Right Turret Teeth rotated", convert_teeth_to_degrees(find_teeth_rotated(rcan1.getAbsolutePosition().getValueAsDouble(), rcan2.getAbsolutePosition().getValueAsDouble())));
  }

  public double convert_teeth_to_degrees(double teeth_rotated){
    double degrees = ((teeth_rotated/100.0) * 360);
    return degrees;
  }

  public double find_teeth_rotated(double m1, double m2) {
    double val =  (306*(m1-m2));
    double t = (val%306+306)%306;
    return t;
  }
}
// done