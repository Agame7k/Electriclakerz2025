package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevatorMotorMaster;
    private SparkMax elevatorMotorSlave;
    private SparkMaxConfig SparkMaxConfigMaster;
    private SparkMaxConfig SparkMaxConfigSlave;
    private DigitalInput elevatorTop;
    private DigitalInput elevatorBottom;
  public ElevatorSubsystem() {
    elevatorMotorMaster = new SparkMax(15, MotorType.kBrushless);
    elevatorMotorSlave = new SparkMax(16, MotorType.kBrushless);
    SparkMaxConfigMaster = new SparkMaxConfig();
    SparkMaxConfigMaster.idleMode(IdleMode.kBrake);
    elevatorMotorMaster.configure(SparkMaxConfigMaster, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    SparkMaxConfigSlave = new SparkMaxConfig();
    SparkMaxConfigSlave.idleMode(IdleMode.kBrake);
    SparkMaxConfigSlave.follow(elevatorMotorMaster);
    elevatorMotorSlave.configure(SparkMaxConfigSlave, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    elevatorTop = new DigitalInput(0);
    elevatorBottom = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator Top", !elevatorTop.get());
    SmartDashboard.putBoolean("Elevator Bottom", !elevatorBottom.get());
  }
  public void SetElevatorSpeed(double speed){
    if (speed > 0 && elevatorTop.get()){
      elevatorMotorMaster.set(0);
    } else if (speed < 0 && elevatorBottom.get()){
      elevatorMotorMaster.set(0);
    } else {
      elevatorMotorMaster.set(speed);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
    
}
