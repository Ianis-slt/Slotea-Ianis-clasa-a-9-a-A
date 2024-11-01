package org.firstinspires.ftc.teamcode.drive.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config; // Permite configurarea din dashboard
import com.acmerobotics.roadrunner.control.PIDCoefficients; // Coeficienți PID pentru controlul mișcării
import com.acmerobotics.roadrunner.drive.DriveSignal; // Structură pentru semnalele de mișcare
import com.acmerobotics.roadrunner.drive.MecanumDrive; // Baza de date pentru un robot cu mecanum drive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower; 
import com.acmerobotics.roadrunner.followers.TrajectoryFollower; // Clasă pentru urmărirea traiectoriilor
import com.acmerobotics.roadrunner.geometry.Pose2d; // Reprezintă poziția în coordonate 2D
import com.acmerobotics.roadrunner.trajectory.Trajectory; // Reprezintă o traiectorie a robotului
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder; // Construitor pentru traiectorii
import com.acmerobotics.roadrunner.trajectory.constraints.*; 
import com.qualcomm.hardware.lynx.LynxModule; // Hardware Lynx Module (comunicare)
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // Setează orientarea hardware a modulului REV
import com.qualcomm.robotcore.hardware.*; // Importă diverse clase de hardware
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // Unitatea de măsură pentru unghiuri
import org.firstinspires.ftc.teamcode.trajectorysequence.*; // Permite definirea secvențelor de traiectorii
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil; // Funcții utilitare pentru modulul Lynx

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.robot.DriveConstants.*; // Importă constantele definite pentru drive

@Config // Permite configurarea variabilelor prin intermediul dashboard-ului
public class SampleMecanumDrive extends MecanumDrive { // Clasa principală pentru mecanum drive
    // Declarație și inițializare a coeficienților PID pentru translatare și orientare
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1; // Multiplicator pentru deplasarea laterală

    public static double VX_WEIGHT = 1; // Greutate pentru viteza pe axa X
    public static double VY_WEIGHT = 1; // Greutate pentru viteza pe axa Y
    public static double OMEGA_WEIGHT = 1; // Greutate pentru viteza de rotație

    private TrajectorySequenceRunner trajectorySequenceRunner; // Obiect pentru rularea secvențelor de traiectorii

    // Constrângeri de viteză și accelerație
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower; // Obiect pentru urmărirea traiectoriei

    // Declararea motoarelor robotului
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors; // Lista motoarelor pentru gestionare mai ușoară

    private IMU imu; 
    private VoltageSensor batteryVoltageSensor; // Senzorul de voltaj al bateriei pentru compensare

    private List<Integer> lastEncPositions = new ArrayList<>(); // Lista ultimelor poziții ale encoderelor
    private List<Integer> lastEncVels = new ArrayList<>(); // Lista ultimelor viteze ale encoderelor

    
    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        // Inițializare Holonomic PID Follower
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Inițializarea IMU și orientarea pe robot
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        // Inițializarea motoarelor de conducere
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Setează comportamentul motoarelor atunci când puterea este zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Adaugă motoarele în lista generală
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

       
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // Inversarea direcției motoarelor din dreapta pentru a asigura mișcarea corectă
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

   
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

   
    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(Dc
