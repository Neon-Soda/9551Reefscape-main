package frc.robot;

public class Constants {
    public static final double stickDeadband = 0.1;
    
    public static final class OIConstants {
        public static final int chassisControllerPort = 0;
        public static final int subsystemControllerPort = 1;
        public static final int testControllerPort = 2;
        public static final double stickDeadband = 0.05;    
      }

    public static final class Carriage{
        // Chassis in canivore (now plugged to rio)
        public static final String chaasisCanbus = "rio";
        // ELevator + wrist + intake + canrange in rio
        public static final String carriageCanbus = "rio";

        public static final int leftElevatorMotorID = 13;
        public static final int rightElevatorMotorID = 14;

        public static final double elevatorMotorRatio = 12.0;
        public static final double Acc = 27.5;
        public static final double Vel = 10.0;
        public static final double Jer = 0.0;

        public static final double elevatorKP = 3.25; 
        public static final double elevatorKI = 0.0;
        public static final double elevatorKD = 0.625;
        public static final double elevatorKS = 0.030909; 
        public static final double elevatorKV = 1.5;
        public static final double elevatorKA = 0.0275;
        public static final double elevatorKG = 0.785;

        // Larger tolerance on top so wrist can always flip over
        public static final double elevatorTolerance = 0.325;
        public static final double elevatorTopperTolerance = 0.625;

        public static final double elevatorOffSetTolerace = 0.185;
        public static final double elevatorOffSetPosition = -elevatorOffSetTolerace;
        public static final double elevatorL1Position = elevatorOffSetPosition;
        public static final double elevatorL2Position = 1.251;
        public static final double elevatorL3Position = 2.58;
        public static final double elevatorL4Position = 5.2;
        public static final double elevatorAlgaeL1Position = 2.25;
        public static final double elevatorAlgaeL2Position = 3.85;
        public static final double elevatorAlgaeOffSetPosition = 1;
        public static final double elevatorNetPosition = 5.47;
        public static final double elevatorProcessorPosition = 1;

        public static final double minHeight = elevatorOffSetPosition;
        public static final double maxHeight = 5.47;

        public static final double elevatorClimbUpDutyCycleOut = 0.65;
        public static final double elevatorClimbDownDutyCycleOut = -0.45;

        public static final int intakeMotorID = 15;
        public static final double intakeWheelDiameter = 0.1;
        public static final int intakeCanRangeID = 16;
        public static final double intakeVoltage = 0.35;
        public static final double L4HoldingVoltage = -0.06;
        public static final double L2L3HoldingVoltage = -0.015;
        public static final double coralL1ScoreVoltage = 0.65;
        public static final double coralScoreVoltage = 1;
        public static final double coralRegretL4Voltage = -0.325;
        public static final double coralRegretVoltage = -0.125;
        public static final double algaeIntakeVoltage = -0.5;
        public static final double algaeHoldingVoltage = -0.15;
        public static final double algaeTransportVoltage = -0.25;
        public static final double algaeScoreVoltage = 1;

        // IMPORTANT: CANrange value to need retune in different environments
        public static final double intakeCanRangeDetectLowerLimit = 6500;
        public static final double intakeCanRangeDetectUpperLimit = 8500;
        

        public static final int wristMotorID = 17;
        public static final int wristEncoderID = 18;
        public static final double wristMotorRatio = 15.0;
        public static final double wristAcc = 5.0;
        public static final double wristVel = 1.0;
        public static final double wristJer = 0.0;

        public static final double wristKP = 25; 
        public static final double wristKI = 0.0; 
        public static final double wristKD = 5; 
        public static final double wristKS = 0.85; 
        public static final double wristKV = 3.0; 
        public static final double wristKA = 0.02;
        public static final double wristKG = 0.85;

        public static final double wristTolerance = 0.075;

        public static final double wristOffSetPosition = -0.025;
        public static final double wristLiftPosition = 0.111;
        public static final double wristL2L3Position = 0.178;
        public static final double wristL4Position = 0.35;
        public static final double wristAlgaePosition = 0.938;
        public static final double wristAlgaeLiftPosition = 0.65;
        public static final double wristAlgaeOffSetPosition = 0.62;
        public static final double wristNetPosition = 0.6;
        public static final double wristNetScoringPosition = 0.3;
        public static final double wristProcessorPosition = 1.125;

        public static final double minRotation = -0.025;
        public static final double maxRotation = 1.1;
    }

    public static final class VisionConstants {
        public static final String limeLightName = "";
        public static final int[] targetAprilTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

        // All the target values are in degrees
        public static final double aprilTagCentricTX = 0;
        public static final double aprilTagCentricTY = -3.55;
        public static final double aprilTagCentricTA = 34.581;
        public static final double centricTolerance = 0.15;

        // All the offsets are in meters
        public static final double limeLightUpOffset = 0.3;
        public static final double limeLightFowardOffset = 0.2;
        public static final double limeLightSideOffset = 0.43;

        public static final double bumperThickness = 0.08;
    }
}
