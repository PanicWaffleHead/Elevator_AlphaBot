package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;


public class ArmHardware implements ArmIO {

    public static final class ArmHardwareConstants {
        private static final int ARM_MOTOR_ID = 8;
        private static final double P_VALUE = 0.1;
        private static final double I_VALUE = 0;
        private static final double D_VALUE = 0.1;
        private static final double P_VALUE_VELOCITY = 0.0001;
        private static final double I_VALUE_VELOCITY = 0;
        private static final double D_VALUE_VELOCITY = 0;
        private static final double FEEDFORWARD_VALUE = 1.0 / 917;
        private static final double OUTPUTRANGE_MIN_VALUE = -1;
        private static final double OUTPUTRANGE_MAX_VALUE = 1;
        private static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(1); 
        private static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.8);
        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.4);
        private static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;
    }

    public SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private SparkClosedLoopController armClosedLoopController;
    private RelativeEncoder armEncoder;

    public ArmHardware() {
        armMotor = new SparkMax(ArmHardwareConstants.ARM_MOTOR_ID, MotorType.kBrushless);

        armMotorConfig = new SparkMaxConfig();

        armClosedLoopController = armMotor.getClosedLoopController();

        armEncoder = armMotor.getEncoder();

        armMotorConfig.encoder
            .positionConversionFactor(ArmHardwareConstants.METERS_PER_REVOLUTION)
            .velocityConversionFactor(ArmHardwareConstants.METERS_PER_REVOLUTION / 60); 
            
        armMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ArmHardwareConstants.P_VALUE, ClosedLoopSlot.kSlot0)
            .i(ArmHardwareConstants.I_VALUE, ClosedLoopSlot.kSlot0)
            .d(ArmHardwareConstants.D_VALUE, ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1)
            .p(ArmHardwareConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .i(ArmHardwareConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .d(ArmHardwareConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
            .velocityFF(ArmHardwareConstants.FEEDFORWARD_VALUE, ClosedLoopSlot.kSlot1) 
            .outputRange(ArmHardwareConstants.OUTPUTRANGE_MIN_VALUE, ArmHardwareConstants.OUTPUTRANGE_MAX_VALUE, ClosedLoopSlot.kSlot1);

        armMotorConfig.closedLoop.maxMotion
            .maxVelocity(ArmHardwareConstants.MAX_VEL.in(MetersPerSecond))
            .maxAcceleration(ArmHardwareConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))
            .allowedClosedLoopError(ArmHardwareConstants.ALLOWED_SETPOINT_ERROR.in(Meters));

        armMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);

        armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPosition(double position) {
        
    }

    @Override
    public void setSpeed(double speed) {
        armClosedLoopController.setReference(speed, SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        armMotor.set(percentOutput);
    }
}
