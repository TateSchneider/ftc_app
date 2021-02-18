package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;


@Autonomous(name = "CleanAuto", group = "comp")


/*******************************************************************************
*                              COMP PROGRAM: START                             *
*******************************************************************************/

// INITILIZATION
public class CleanAuto extends LinearOpMode {

// Declaring Motors
    private     DcMotorEx     rf;         //port 0
    private     DcMotorEx     lf;         //port 1
    private     DcMotorEx     rb;         //port 2
    private     DcMotorEx     lb;         //port 3
    private     DcMotor     rs;         //port 0
    private     DcMotorEx     rl1;        //port 1
    private     DcMotorEx     rl2;        //port 2
    private     DcMotor     as;         //port 3

    private     Servo       _rf;        //port 0
    private     Servo       ts;         //port 0
    private     Servo       bs;         //port 1
    
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    boolean foundRed = false;
    boolean foundWhite = false;
    
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double heading;
    boolean turned180 = false;
    boolean didStrafe = false;
    boolean isStraight = false;
    
    boolean isShooting = false;
    PIDBase pidRotate, pidDrive; //add
    Orientation lastAngles = new Orientation(); //add
    double globalAngle, power = .30, correction, rotation; //add
    
    boolean singleStack = false;
    boolean quadStack = false;
    
//Declaring Camera Variables 
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    private static final String LABEL_FIRST_ELEMENT = "Quad Stack";
    private static final String LABEL_SECOND_ELEMENT = "Single Stack";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY = "ARxaOAX/////AAABmR91q9ci+kNYqGb/NElhuhBQa5klidYZ5jKk5hFYJ6qAQOtCGKSEZXn1qYawipXKEEpJh+vP3GNnOUvabO2blz4vkymDnu8LUocLc6/rMpQdLwBt80JVdgWWkd/4j1DmwDdRRP4f/jP78furjgexjT7HgmC37xLP+msr78zAeWwkrsT2X1yjnL6nyiGcRKlBw6+EcUIcZYiiuXwbILds8rl4Fu7AuecLaygDft6XIUFg/qQm51UF45l5pYT8AoNTUhP9GTksKkmHgde7iGlo3CfIYu9QanjPHreT/+JZLJWG22jWC7Nnzch/1HC6s3s2jzkrFV6sRVA4lL9COLIonjRBYPhbxCF06c5fUMy9sj/e";

    @Override
    
    public void runOpMode() {
        telemetry.addData("Stat", "Initializing...");
        telemetry.update();
        
        startInit();
            
        telemetry.addData("Stat", "Start Program");
        telemetry.update();
        
    //Waiting for start via Player
        waitForStart();
        
    //Beginning Loop for Program
        if (opModeIsActive()) {
            
            pidTurn(90, 0.3, 0.1, 0.001, 0.000005);
            /*moveInches(50, 800);
            pidSenseLine(0.3, 0, 0.3, "white", 0.1, 0.001, 0.000005);
            moveInches(-10, 500);
            //powerShot(200);
            sleep(1000);
            
            forceStop();
            
            if(quadStack) {
                encoders("n");
                pidSenseLine(0.15, 0, 0.15, "white", 0.1, 0.001, 0.000005);
                moveInches(48, 700);
                encoders("n");
                turn(0.5, 94);
                moveInches(-30, 700);
                encoders("n");
                dropWobble();
                moveInches(38, 700);
                encoders("n");
                turn(0.4, 180);
                senseLine("white", 0.3);
                forceStop();
            } else if(singleStack) {
                encoders("n");
                pidSenseLine(0.15, 0, 0.15, "white", 0.1, 0.001, 0.000005);
                moveInches(24, 700);
                turn(0.5, 130);
                moveInches(2, 500);
                dropWobble();
                moveInches (5, 500);
                turn(0.5, 180);
                senseLine("white", 0.3);
                forceStop();
            } else {
                encoders("n");
                pidSenseLine(0.15, 0, 0.15, "white", 0.1, 0.001, 0.000005); // 0.25
                senseLine("red", 0.3);
                turn(0.33, 90);
                moveInches(-24, 600);
                dropWobble();
                as.setPower(1);
                sleep (50);
            }
            as.setPower(-1);
            sleep(400);
            forceStop(); */
        }
    }


//Method to Completely Stop ALL Robot Movement
    private void forceStop() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        as.setPower(0);
        rs.setPower(0);
        rl1.setPower(0);
        rl2.setPower(0);
        bs.setPosition(0);
        _rf.setPosition(0.8);
    }


//Method to Halt Robot Movement
    public void stopRobot() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
    
    void powerShot (int inches) {
        double lengthUsingInches = -inches;
       
        rl1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rl1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double calcPosition = lengthUsingInches * (100* 280/(16.9646003294*4 *8.8 * 1.0555555556));
        int setPosition = (int) Math.round(calcPosition);

        rl1.setTargetPosition(setPosition);
        rl2.setTargetPosition(setPosition);

        rl1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rl2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rl1.setVelocity(2060); //1850 o 1880
        rl2.setVelocity(2060); //1850 0 1880
        
        sleep(1900);

        while (opModeIsActive() && (Math.abs(setPosition) >= Math.abs(rl1.getCurrentPosition()))) {
            telemetry.addData("position", rl2.getCurrentPosition());
            telemetry.addData("is at target", !rl2.isBusy());
            telemetry.update();
            
            _rf.setPosition(0.4);
            sleep(300);
            _rf.setPosition(0.8);
            sleep(300); 
        
            rs.setPower(1);
        }

        rl1.setPower(0);
        rl1.setPower(0);
        
        rl1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rl1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rl2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.addData("position", rl2.getCurrentPosition());
        telemetry.addData("is at target", !rl2.isBusy());
        telemetry.update();
                
     }
    
    public void encoders(String answer) {
        if (answer.equals("y")) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (answer.equals("n")) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } 
    }
    
    public void dropWobble() {
        bs.setPosition(0.5);
        sleep(100);
        ts.setPosition(0.5);
        sleep(400);
    }
    

//Method to Find & Move to the White Line
    void senseLine(String color, double speed) {
        //Needed (non-changing) Variables
        final float[] hsvValues = new float[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        foundRed = false;
        foundWhite = false;
        int countRed = 0;
        int countWhite = 0;

        //If the "foundRed" Boolean is False, Run Loop
        while ((!foundRed && !foundWhite) && opModeIsActive()) {
            //Needed (updating) Variables
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            //If-Else-If Statement to Drive Forward in a Straight Line
            if (speed > 0) {
                if (heading < -0.1  && heading > -90){
                    lf.setPower(speed - (0.025 * heading));
                    lb.setPower(speed - (0.025 * heading));
                    rf.setPower(speed + (0.025 * heading));
                    rb.setPower(speed + (0.025 * heading));
                }else if (heading > 0.1 && heading < 90){
                    lf.setPower(speed + (0.025 * heading));
                    lb.setPower(speed + (0.025 * heading));
                    rf.setPower(speed - (0.025 * heading));
                    rb.setPower(speed - (0.025 * heading));
                } else {
                    lf.setPower(speed);
                    lb.setPower(speed);
                    rf.setPower(speed);
                    rb.setPower(speed);
                }
            } else if (speed < 0) {
                //May need to switch the positives and negatives if gyro seems not too work
                if (heading < -0.1  && heading > -90){
                    lf.setPower(speed - (0.025 * heading));
                    lb.setPower(speed - (0.025 * heading));
                    rf.setPower(speed + (0.025 * heading));
                    rb.setPower(speed + (0.025 * heading));
                }else if (heading > 0.1 && heading < 90){
                    lf.setPower(speed + (0.025 * heading));
                    lb.setPower(speed + (0.025 * heading));
                    rf.setPower(speed - (0.025 * heading));
                    rb.setPower(speed - (0.025 * heading));
                } else {
                    lf.setPower(speed);
                    lb.setPower(speed);
                    rf.setPower(speed);
                    rb.setPower(speed);
                }
            }

            if (color.equals("red")) {
                if (colors.alpha < 0.2) {
                    stopRobot();
                    foundRed = true;
                }
                countRed++;
            } else if (color.equals("white")) {
                if (colors.alpha > 0.5) {
                    stopRobot();
                    foundWhite = true;
                }
                countWhite++;
            } else {
                stopRobot();
            }
        }
    }
    
    void pidSenseLine(double speed, double time, double finalSpeed, String color, double kP, double kI, double kD) {
        double currentMili = System.currentTimeMillis();
        pidDrive = new PIDBase(kP, kI, kD);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        final float[] hsvValues = new float[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        foundRed = false;
        foundWhite = false;
        int countRed = 0;
        int countWhite = 0;
        
        while ((currentMili + time > System.currentTimeMillis()) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());

            lf.setPower(speed - correction);
            lb.setPower(speed - correction);
            rf.setPower(speed + correction);
            rb.setPower(speed + correction);
        }
     
        while ((!foundRed && !foundWhite) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            lf.setPower(finalSpeed - correction);
            lb.setPower(finalSpeed - correction);
            rf.setPower(finalSpeed + correction);
            rb.setPower(finalSpeed + correction);
        
            if (color.equals("red")) {
                if (colors.alpha < 0.2) {
                    stopRobot();
                    foundRed = true;
                }
                countRed++;
            } else if (color.equals("white")) {
                if (colors.alpha > 0.5) {
                    stopRobot();
                    foundWhite = true;
                }
                countWhite++;
            } else {
                stopRobot();
            }
            
        }
    }
    
    public void pidTurn(int angleMeasure, double finalSpeed, double kP, double kI, double kD) {
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turned180 = false;
        pidDrive = new PIDBase(kP, kI, kD);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        
        while (!turned180 && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

            telemetry.addData("heading", heading);
            telemetry.update();
            
            if (heading <= angleMeasure + 1 && heading >= angleMeasure - 1) {
                stopRobot();
                turned180 = true;
            } else if (heading >= (0.9 * angleMeasure) && heading < (angleMeasure - 1)) {
                lf.setPower(-0.5 * (finalSpeed - correction));
                lb.setPower(-0.5 * (finalSpeed - correction));
                rf.setPower(0.5 * (finalSpeed - correction));
                rb.setPower(0.5 * (finalSpeed - correction));
            } else {
                lf.setPower(-(finalSpeed - correction));
                lb.setPower(-(finalSpeed - correction));
                rf.setPower(finalSpeed - correction);
                rb.setPower(finalSpeed - correction);
            }
        }
    }


    void turn(double speed, int angleMeasure) {
        turned180 = false;
        while (opModeIsActive() && !turned180) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

            telemetry.addData("heading", heading);
            telemetry.update();

            if (heading <= angleMeasure + 1 && heading >= angleMeasure - 1) {
                stopRobot();
                turned180 = true;
            } else if (heading >= (0.9 * angleMeasure) && heading < (angleMeasure - 1)) {
                rf.setPower(0.5 * speed);
                lf.setPower(-0.5 *speed);
                rb.setPower(0.5 * speed);
                lb.setPower(0.5 * -speed);
            } else {
                rf.setPower(speed);
                lf.setPower(-speed);
                rb.setPower(speed);
                lb.setPower(-speed);
            }
        }
    }
    
    void moveInches(double lengthUsingInches, double velocity) {
        
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        double calcPosition = lengthUsingInches * (100* 280/(16.9646003294*4 *8.8 * 1.0555555556));
        int setPosition = (int) Math.round(calcPosition);

        lf.setTargetPosition(setPosition);
        rf.setTargetPosition(setPosition);
        lb.setTargetPosition(setPosition);
        rb.setTargetPosition(setPosition);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setVelocity(velocity);
        rf.setVelocity(velocity);
        lb.setVelocity(velocity);
        rb.setVelocity(velocity);

        while (opModeIsActive() && lf.isBusy()) {
            telemetry.addData("position", lf.getCurrentPosition());
            telemetry.addData("is at target", !lf.isBusy());
            telemetry.update();
        }

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private void initVuforia() {
        
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    
    
    
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }


    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }


    private void rotate(int degrees, double power) {
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {
                lf.setPower(power);
                rf.setPower(-power);
                lb.setPower(power);
                rb.setPower(-power);
                sleep(100);
            }

            do{
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                lf.setPower(-power);
                rf.setPower(power);
                lb.setPower(-power);
                rb.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else {
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                lf.setPower(-power);
                rf.setPower(power);
                lb.setPower(-power);
                rb.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

            rf.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            lb.setPower(0);

            rotation = getAngle();
    
            sleep(500);
            resetAngle();
        }
    }
    
    public void startInit() {
        rf  =   hardwareMap.get(DcMotorEx.class, "rightFront");
        lf  =   hardwareMap.get(DcMotorEx.class, "leftFront");
        rb  =   hardwareMap.get(DcMotorEx.class, "rightBack");
        lb  =   hardwareMap.get(DcMotorEx.class, "leftBack");
        
        rs  =   hardwareMap.dcMotor.get("ringScoop");
        rl1 =   hardwareMap.get(DcMotorEx.class, "ringLaunch1");
        rl2 =   hardwareMap.get(DcMotorEx.class, "ringLaunch2");
        as  =   hardwareMap.dcMotor.get("armString");
        
        _rf =   hardwareMap.servo.get("ringFling");
        ts  =   hardwareMap.servo.get("topServo");
        bs  =   hardwareMap.servo.get("bottomServo");
        
        // Extra Motor Steps
        lf.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse
        lb.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse
        
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        //Initialize Camera
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        
        //Camera Detection
        ts.setPosition(0.99);
        bs.setPosition(0);
        _rf.setPosition(0.8);
        sleep(200);
        
        //as.setPower(1);
        //sleep(500);
        forceStop();
        
        
        while (!opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# of Rings", updatedRecognitions.size()); //# Object Detected
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("Label"), recognition.getLabel());
                        if(recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                            singleStack = true;
                        } else if(recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                            quadStack = true;
                        }
                    }
                    telemetry.update();
                }
            }
        }
        imu.initialize(parameters);
        
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
