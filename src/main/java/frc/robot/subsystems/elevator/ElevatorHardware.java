package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorConstants;

public class ElevatorHardware implements ElevatorIO {
    public SparkMax elevatorRightMotor, elevatorLeftMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController rightClosedLoopController;
    private SparkClosedLoopController leftClosedLoopController;
    private RelativeEncoder rightEncoder, leftEncoder;
    
    public ElevatorHardware() {
        elevatorRightMotor = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorLeftMotor = new SparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless)
        rightClosedLoopController = elevatorRightMotor.getClosedLoopController();
        leftClosedLoopController = elevatorLeftMotor.getClosedLoopController();

        rightEncoder = elevatorRightMotor.getEncoder();
        leftEncoder = elevatorLeftMotor.getEncoder();


        motorConfig = new SparkMaxConfig();

        motorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        elevatorLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorLeftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    public void setSpeed(double speed) {

    }

    public void setPosition(double position) {

    }

    public double getSpeed() {
    
    }

    public double getPosition() {

    }
}
