package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

@TeleOp(name="Robot: camera_test", group="Robot")

public class camera_test extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    Limelight3A limelight;

    @Override
    public void runOpMode() {

        DcMotor frontL = hardwareMap.get(DcMotor.class, "flm");
        DcMotor backL = hardwareMap.get(DcMotor.class, "blm");
        DcMotor frontR = hardwareMap.get(DcMotor.class, "frm");
        DcMotor backR = hardwareMap.get(DcMotor.class, "brm");
        DcMotorEx spin = hardwareMap.get(DcMotorEx.class, "spin"); // + is left - is right

        IMU imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!


        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);

        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        waitForStart();
        runtime.reset();

        int speeddiv = 1;
        int AprilTag = 0;
        Integer[] redTags = {30,31,32,33,34,35,36,37};
        Integer[] blueTags = {30,31,32,33,34,35,36,37};
        double tx = 0;
        double ty = 0;
        double ta = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * 0.7;
            double lateralx = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double axialy = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(axialy) + Math.abs(lateralx) + Math.abs(yaw), 1);

            double leftFrontPower = (axialy + lateralx + yaw) / denominator;
            double leftBackPower = (axialy - lateralx + yaw) / denominator;
            double rightFrontPower = (axialy - lateralx - yaw) / denominator;
            double rightBackPower = (axialy + lateralx - yaw) / denominator;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }

                tx = result.getTx();
                ty = result.getTy(); // How far up or down the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)
                List<FiducialResult> tags = result.getFiducialResults();

                if (!tags.isEmpty()) {
                    for (FiducialResult tag : tags) {
                        AprilTag = tag.getFiducialId();
                        telemetry.addData("Target ID", AprilTag);
                    }
                }
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            /*
            if (tx > 0 && Arrays.asList(redTags).contains(AprilTag)) {
                spin.setPower(-0.02);
            } else if (tx < 0 && Arrays.asList(redTags).contains(AprilTag)) {
                spin.setPower(0.02);
//            } else if (!Arrays.asList(redTags).contains(AprilTag)) {
//                spin.setTargetPosition(1);
//                if (spin.getCurrentPosition() < spin.getTargetPosition() && spin.getTargetPosition() == 1) {
//                    spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    spin.setPower(0.02);
//                } else if (spin.getCurrentPosition() > spin.getTargetPosition() && spin.getTargetPosition() == 1) {
//                    spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    spin.setPower(-0.02);
//                } else {
//                    spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    spin.setPower(0);
//                    spin.setTargetPosition(0);
//                }
            } else if (!Arrays.asList(redTags).contains(AprilTag)) {
                spin.setPower(0);
            }
             */

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
            telemetry.update();
        }
    }
}