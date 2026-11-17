package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Robot: opmode_1", group="Robot")

public class opmode_1 extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor frontL = hardwareMap.get(DcMotor.class, "flm");
        DcMotor backL = hardwareMap.get(DcMotor.class, "blm");
        DcMotor frontR = hardwareMap.get(DcMotor.class, "frm");
        DcMotor backR = hardwareMap.get(DcMotor.class, "brm");

        DcMotorEx launch = hardwareMap.get(DcMotorEx.class, "launch");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        Servo DTI = hardwareMap.get(Servo.class, "dti");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DTI.setPosition(0.45);

        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int speeddiv = 4;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial    = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral  =  gamepad1.left_stick_x;
            double yaw      =  gamepad1.right_stick_x * 1.1;
            double lateralx = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double axialy   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(axialy) + Math.abs(lateralx) + Math.abs(yaw), 1);

            double leftFrontPower = (axialy + lateralx + yaw) / denominator;
            double leftBackPower = (axialy - lateralx + yaw) / denominator;
            double rightFrontPower = (axialy - lateralx - yaw) / denominator;
            double rightBackPower = (axialy + lateralx - yaw) / denominator;

            lateralx = lateralx * 1.1;

            if (gamepad1.y && speeddiv == 4) {
                speeddiv = 2;
            }

            if (gamepad1.y && speeddiv == 2) {
                speeddiv = 8;
            }

            if (gamepad1.y && speeddiv == 8) {
                speeddiv = 4;
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.dpad_up) {
                intake.setPower(100);
            }

            if (gamepad1.dpad_down) {
                intake.setPower(0);
            }

            if (gamepad1.a) {
                launch.setVelocity(1200);
            }

            if (gamepad1.b) {
                launch.setVelocity(0);
            }

            if (gamepad1.x) {
                DTI.setPosition(0);
                sleep(3000);
                DTI.setPosition(0.45);
            }

            // Send calculated power to wheels
            frontL.setPower(leftFrontPower / speeddiv);
            frontR.setPower(rightFrontPower / speeddiv);
            backL.setPower(leftBackPower / speeddiv);
            backR.setPower(rightBackPower / speeddiv);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial  ", "%4.2f", axial);
            telemetry.addData("lateral  ", "%4.2f", lateral);
            telemetry.addData("yaw  ", "%4.2f", yaw);
            telemetry.addData("launch speed", launch.getVelocity());
            telemetry.addData("speed", speeddiv);
            telemetry.update();

            dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
            dashboardTelemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            dashboardTelemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            dashboardTelemetry.addData("Axial  ", "%4.2f", axial);
            dashboardTelemetry.addData("lateral  ", "%4.2f", lateral);
            dashboardTelemetry.addData("yaw  ", "%4.2f", yaw);
            dashboardTelemetry.addData("launch speed", launch.getVelocity());
            dashboardTelemetry.addData("speed", speeddiv);
            dashboardTelemetry.update();

        }
    }}