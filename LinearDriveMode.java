// Importuri pentru clase și funcții necesare
package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs; 

import com.acmerobotics.dashboard.FtcDashboard; // Dashboard pentru vizualizare în timp real
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry; // Permite afișarea multiplă a datelor de telemetrie
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Clasă de bază pentru modurile de op Linear
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; 
import com.acmerobotics.roadrunner.geometry.Pose2d; // Clase pentru poziția 2D (x, y, orientare)

import org.firstinspires.ftc.teamcode.drive.robot.Robot; // Importă clasa robotului definită de utilizator


@TeleOp(name="MecanumDriveMode", group="Linear OpMode")
public class LinearDriveMode extends LinearOpMode { // Definirea clasei pentru modul Linear
    private Robot robot = null; // Instanță de robot, inițializată cu null
    int direction = 1; // Variabilă pentru direcție, inițializată la 1
    double servoPosSlides = 0.5; // Poziția implicită a servomotorului pentru slides
    double servoPosGrippy = 0; // Poziția implicită a servomotorului pentru gripper

 
    public double calculateThrottle(float x) {
        int sign = -1; // Inițializează semnul ca fiind negativ
        if (x > 0) sign = 1; // Dacă valoarea e pozitivă, schimbă semnul în pozitiv
        return sign * 3 * abs(x); 
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing..."); // Afișează mesaj de inițializare pe telemetrie
        telemetry.update(); // Actualizează telemetria

        robot = new Robot(hardwareMap); // Creează o instanță a clasei robot, legată la hardware-ul definit
        while (robot.isInitialize() && opModeIsActive()) { 
            idle(); 
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Setează telemetria
        telemetry.addData(">", "Initialized"); // Afișează mesaj de inițializare completă
        telemetry.update(); // Actualizează telemetria

        waitForStart(); // Așteaptă începerea efectivă 
        if (isStopRequested()) return; // Verifică dacă a fost cerută oprirea

        while (opModeIsActive()) { 
            // Controlează direcția și mișcarea slide-urilor cu butoanele bumpere
            if (gamepad2.left_bumper) { 
                robot.crane.slidesDirection = 1; // Setează direcția slide-ului în sus
                robot.crane.setSlides(5); // Setează slide-urile la o valoare de putere
                if(robot.crane.slideEncoderLastPosition > robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension -= 3.3; // Ajustează extensia slide-ului dacă poziția a scăzut
                }
            } else if (gamepad2.right_bumper) {
                robot.crane.slidesDirection = -1; // Setează direcția slide-ului în jos
                robot.crane.setSlides(5); // Setează slide-urile la o valoare de putere
                if(robot.crane.slideEncoderLastPosition < robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension += 3.3; // Ajustează extensia slide-ului dacă poziția a crescut
                }
            } else {
               robot.crane.setSlides(0); 
            }
            robot.crane.slideEncoderLastPosition = robot.crane.slideEncoder.getVoltage(); // Actualizează ultima poziție a slide-urilor

            // Controlează accelerația macaralei cu triggerele de pe gamepad
            if(gamepad2.left_trigger > 0.1){
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad2.left_trigger); // Micșorează ținta macaralei
            }
            else if(gamepad2.right_trigger > 0.1){
                robot.crane.craneTarget += (int) calculateThrottle(gamepad2.right_trigger); // Mărește ținta macaralei
            }
            // Setează puterea motorului pentru macara
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget));
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget));

            // Controlează direcția gripper-ului cu butoanele A și B de pe gamepad
            if (gamepad2.a) {
                robot.crane.gripperDirection = 1; // Setează gripper-ul pentru a apuca
                robot.crane.setGripper(1); // Activează gripper-ul
            }
            else if (gamepad2.b) {
                robot.crane.gripperDirection = -1; // Setează gripper-ul pentru a elibera
                robot.crane.setGripper(1); // Activează gripper-ul
            }
            else robot.crane.setGripper(0); // Dacă nici A nici B nu sunt apăsate, dezactivează gripper-ul

            // Setează puterea de mișcare a robotului pe baza mișcărilor de pe gamepad
            robot.drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y),(-gamepad1.left_stick_x),(-gamepad1.right_stick_x)));

         
            telemetry.addData("crane target: ", robot.crane.craneTarget);
            telemetry.addData("right trigger: ", gamepad2.right_trigger);
            telemetry.addData("encoder value: ", robot.crane.slideEncoder.getVoltage());
            telemetry.addData("last position ", robot.crane.slideEncoderLastPosition);
            telemetry.addData("slide extension ", robot.crane.slideExtension);
            telemetry.addData("sensor touch: ", robot.crane.slideSensor.isPressed());
            telemetry.update(); // Actualizează telemetria pentru a afișa datele noi
        }
    }
}
