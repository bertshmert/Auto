package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;




/**
 * Created by maryjaneb  on 11/13/2016.
 * <p>
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 * <p>
 * monitor: 640 x 480
 * YES
 */
@Autonomous( name = "SkyStoneNew", group = "Sky autonomous" )
//comment out this line before using
public class NewSkystone extends LinearOpMode {




	HardwareAutoNeoKicked robot = new HardwareAutoNeoKicked();
	private ElapsedTime runtime = new ElapsedTime();

	//0 means skystone, 1 means yellow stone
	//-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
	private static int valMid = -1;
	private static int valLeft = -1;
	private static int valRight = -1;

	private static float rectHeight = .6f / 8f;
	private static float rectWidth = 1.5f / 8f;

	private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
	private static float offsetY = 3f / 20f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

	private static float[] midPos = { 4f / 8f + offsetX, 4f / 8f + offsetY };//0 = col, 1 = row
	private static float[] leftPos = { 2f / 8f + offsetX, 4f / 8f + offsetY };
	private static float[] rightPos = { 6f / 8f + offsetX, 4f / 8f + offsetY };
		//moves all rectangles right or left by amount. units are in ratio to monitor

	private final int rows = 640;
	private final int cols = 480;

	OpenCvCamera phoneCam;




	@Override
	public void runOpMode() throws InterruptedException {


		robot.init( hardwareMap );

		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );


		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );

		//P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
		//phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
		phoneCam = new OpenCvInternalCamera( OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId );//remove this

		phoneCam.openCameraDevice();//open camera
		phoneCam.setPipeline( new StageSwitchingPipeline() );//different stages
		phoneCam.startStreaming( rows, cols, OpenCvCameraRotation.UPRIGHT );//display on RC
		//width, height
		//width = height in this case, because camera is in portrait mode.

		telemetry.addData( "Values", valLeft + "   " + valMid + "   " + valRight );
		telemetry.addData( "Height", rows );
		telemetry.addData( "Width", cols );
		waitForStart();

		runtime.reset();

		while ( robot.mrGyro.isCalibrating() && opModeIsActive() ) {


			telemetry.update();


			//call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);


			if ( valLeft == 255 && valMid == 255 && valRight == 0 ) {


				driveAuto( 0 );
				sleep( 300 );


				driveSideRightUS( .5, 12 );//moves towards skystone
				robot.autoGrab.setPosition( 0.7 );//servo moves in position

				robot.autoArm.setPower( -.5 );//moves side arm down
				sleep( 500 );
				robot.autoArm.setPower( 0 );//moves side to get position

				sleep( 200 );
				driveSideRightEncoder( .1, 200 );//moves to get grip
				robot.autoGrab.setPosition( 0 );//pick up servo
				sleep( 500 );

				robot.autoArm.setPower( 0.7 );//pick up arm
				sleep( 500 );
				robot.autoArm.setPower( 0 );
				sleep( 200 );//

				driveSideLeftEncoder( .7, 400 );//get away from blocks
				sleep( 200 );
				turn( 1 );
				driveBackUS( .8, 55 );//go towards plate
				sleep( 200 );
				driveSideRightEncoder( 0.5, 900 );//drive up to plate//
				sleep( 200 );

				robot.autoArm.setPower( -0.3 );//bring arm down
				sleep( 700 );
				robot.autoArm.setPower( 0 );//set stationary

				robot.autoGrab.setPosition( 0.7 );//let go of block

				robot.autoArm.setPower( 0.3 );//bring up
				sleep( 500 );
				robot.autoArm.setPower( 0 );//stationary
				sleep( 200 );
				driveSideLeftEncoder( .5, 700 );//get away from plate and be ready to go back
				sleep( 200 );
				//to park on tape
				driveForward( 1, 2000 );


			}


			//block in middle
			else if ( valLeft == 255 && valMid == 0 && valRight == 255 ) {

				driveForward( .2, 600 );
				driveAuto( 700 );
				driveAuto( 2100 );

			}


			//block on left
			else {

				driveForward( .3, 1300 );
				driveAuto( 1400 );
				driveBack( 1, 3900 );

			}

		}

	}





	public void driveAuto( int offset ) {

		/*driveSideRightUS( 0.7, 18);//moves towards skystone
		driveSideRightUS( 0.2, 3 );*/
		sleep( 200 );
		driveSideRightUS( .5,12 );//go towards block
		sleep( 200 );
		robot.autoGrab.setPosition( 0.7 );//servo moves in position
		robot.autoArm.setPower( -.5 );//moves side arm down
		sleep( 500 );
		robot.autoArm.setPower( 0 );//moves side to get position
		sleep( 200 );
		driveSideRightEncoder( .1, 200);//moves to get grip
		robot.autoGrab.setPosition( 0 );//pick up servo
		sleep( 500 );
		robot.autoArm.setPower( 0.7 );//pick up arm
		sleep( 500 );
		robot.autoArm.setPower( 0 );
		sleep( 200 );//
		driveSideLeftEncoder( .5, 400);//get away from blocks
		sleep( 200 );
		driveBackUS( .6, 40);//go towards plate
		sleep( 200 );
		driveSideRightUS( 0.5, 10);//drive up to plate
		sleep( 200 );

		robot.autoArm.setPower( -0.3 );//bring arm down
		sleep( 700 );
		robot.autoArm.setPower( 0 );//set stationary

		robot.autoGrab.setPosition( 0.7 );//let go of block

		robot.autoArm.setPower( 0.3 );//bring up
		sleep( 500 );
		robot.autoArm.setPower( 0 );//stationary

		driveSideLeftEncoder( .5, 650);//get away from plate and be ready to go back
		sleep( 200 );
		//negative turns right and positive turns left
		turn( 1 );//allign
		sleep( 200 );
		//to go back for 2nd block
	turn( 3 );
		driveForward( 1,7400 + offset);
		sleep( 400);

	}




	public void driveForward( double speed, int Target ) {

		//RESET encoders
		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		//set target
		robot.rightFront.setTargetPosition( -Target );
		robot.leftBack.setTargetPosition( -Target );
		robot.rightBack.setTargetPosition( -Target );
		robot.leftFront.setTargetPosition( -Target );
		//run to position
		robot.rightFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		runtime.reset();
		//set speed
		robot.rightFront.setPower( speed );
		robot.leftBack.setPower( speed );
		robot.rightBack.setPower( speed );
		robot.leftFront.setPower( speed );
		//update telemetry to show positions
		while ( robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && opModeIsActive() ) {
		}
		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );
		//turn on encoders
		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

	}




	public void driveForwardUCS( double speed ) {

		robot.rightFront.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		/*while ( ( robot.colorSensor.blue() == 0 ) && opModeIsActive() ) {

			robot.zAccumulated = robot.mrGyro.getIntegratedZValue();

			robot.rightFront.setPower( -speed );
			robot.leftBack.setPower( -speed );
			robot.rightBack.setPower( -speed );
			robot.leftFront.setPower( -speed );

			telemetry.addData( "step", "driving forward" );
			telemetry.update();
		}
		telemetry.addData( "step", "done driving forward" );*/

		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );

	}




	public void driveBack( double speed, int Target ) {

		//RESET encoders
		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		//set target
		robot.rightFront.setTargetPosition( Target );
		robot.leftBack.setTargetPosition( Target );
		robot.rightBack.setTargetPosition( Target );
		robot.leftFront.setTargetPosition( Target );
		//run to position
		robot.rightFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		runtime.reset();
		//set speed
		robot.rightFront.setPower( -speed );
		robot.leftBack.setPower( -speed );
		robot.rightBack.setPower( -speed );
		robot.leftFront.setPower( -speed );
		//update telemetry to show positions
		while ( robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && opModeIsActive() ) {
		}
		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );
		//turn on encoders
		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

	}




	public void driveBackUS( double speed, double distance ) {

		robot.rightFront.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		runtime.reset();

		double target = robot.mrGyro.getIntegratedZValue();
		while ( ( robot.rangeSensor.rawUltrasonic() > distance ) && opModeIsActive() ) {

			robot.rightFront.setPower( speed );
			robot.leftBack.setPower( speed  );
			robot.rightBack.setPower( speed  );
			robot.leftFront.setPower( speed  );
//			correction = robot.freeWheel.getCurrentPosition() - f;

		}


		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );

	}




	public void driveSideRightUS( double speed, double distance ) {

		//RESET encoders
		//  robot.freeWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//set target

		//run to position

		runtime.reset();

		//set speed

		double correction = 0;
//        double f = robot.freeWheel.getCurrentPosition();
//        telemetry.addData("freeWheel",robot.freeWheel.getCurrentPosition());
        /*
        while(ftime < 20){
            telemetry.addData("freeWheel",robot.freeWheel.getCurrentPosition());
            telemetry.update();
            sleep(1000);
            ftime = ftime + 1;
        }

         */
		double target = robot.mrGyro.getIntegratedZValue();
		while ( ( robot.rangeSensor2.rawUltrasonic() > distance ) && opModeIsActive() ) {

			robot.zAccumulated = robot.mrGyro.getIntegratedZValue();
//            telemetry.addData("freeWheel",robot.freeWheel.getCurrentPosition());
			telemetry.addData( "gyro", robot.zAccumulated );
//			telemetry.update();

			robot.rightFront.setPower( speed - correction / 1000 + ( robot.zAccumulated - target ) * 0 / 500 );
			robot.leftBack.setPower( speed - correction / 1000 - ( robot.zAccumulated - target ) * 0 / 500 );
			robot.rightBack.setPower( -speed - correction / 1000 - ( robot.zAccumulated - target ) * 0 / 500 );
			robot.leftFront.setPower( -speed - correction / 1000 + ( robot.zAccumulated - target ) * 0 / 500 );
			//sleep(10);
//			correction = robot.freeWheel.getCurrentPosition() - f;

		}


		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );

	}




	public void driveSideRightEncoder( double speed, int target ) {
		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		robot.rightFront.setTargetPosition( target );
		robot.leftBack.setTargetPosition( target );
		robot.leftFront.setTargetPosition( -target );
		robot.rightBack.setTargetPosition( -target );

		robot.rightFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		runtime.reset();

		while( robot.rightFront.isBusy() && robot.leftBack.isBusy() && robot.leftFront.isBusy() && robot.rightBack.isBusy() ) {

			robot.rightFront.setPower( speed );
			robot.leftBack.setPower( speed );
			robot.rightBack.setPower( -speed );
			robot.leftFront.setPower( -speed );
		}

		//  robot.lrwheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//RESET encoders
		//  robot.freeWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//  robot.lrwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//set target

		//  robot.lrwheel.setTargetPosition(Target);


		//run to position

		// robot.lrwheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		//runtime.reset();

		//set speed

		double correction = 0;
		// double f = robot.freeWheel.getCurrentPosition();
		int ftime = 0;
		//telemetry.addData("freeWheel",robot.freeWheel.getCurrentPosition());
        /*
        while(ftime < 20){
            telemetry.addData("freeWheel",robot.freeWheel.getCurrentPosition());
            telemetry.update();
            sleep(1000);
            ftime = ftime + 1;
        }

         */
		//double target = robot.mrGyro.getIntegratedZValue();


		//while(robot.lrwheel.getCurrentPosition() > - Target){

		// robot.zAccumulated = robot.mrGyro.getIntegratedZValue();
		//    telemetry.addData("freeWheel",robot.freeWheel.getCurrentPosition());
		// telemetry.addData("gyro",robot.zAccumulated);
		//  telemetry.update();


		//   robot.rightFront.setPower(speed - correction/1000 + (robot.zAccumulated - target)*0/500);
		//  robot.leftBack.setPower(speed - correction/1000 - (robot.zAccumulated - target)*0/500);
		// robot.rightBack.setPower(-speed - correction/1000 - (robot.zAccumulated - target)*0/500);
		//  robot.leftFront.setPower(-speed - correction/1000 + (robot.zAccumulated - target)*0/500);
		//sleep(10);
		//     correction = robot.freeWheel.getCurrentPosition() - f;


		//}

		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );

	}




	public void driveSideLeftEncoder( double speed, int target ) {

		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		robot.rightBack.setTargetPosition( target );
		robot.leftFront.setTargetPosition( target );
		robot.rightFront.setTargetPosition( -target );
		robot.leftBack.setTargetPosition( -target );

		robot.rightBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.rightFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		runtime.reset();

		while( robot.rightBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.leftBack.isBusy() ) {
			robot.rightFront.setPower( -speed );
			robot.leftBack.setPower( -speed );
			robot.rightBack.setPower( speed );
			robot.leftFront.setPower( speed );
		}

//		robot.lrwheel.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
//
//		//RESET encoders
//		robot.freeWheel.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
//		robot.lrwheel.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
//
//		//set target
//		robot.lrwheel.setTargetPosition( -Target );
//
//		//run to position
//		robot.lrwheel.setMode( DcMotor.RunMode.RUN_TO_POSITION );
//		runtime.reset();
//
//		//set speed
//		double correction = 0;
//		double f = robot.freeWheel.getCurrentPosition();
//		telemetry.addData( "freeWheel", robot.freeWheel.getCurrentPosition() );
//		telemetry.addData( "lrwheel", robot.lrwheel.getCurrentPosition() );
//
//		double target = robot.mrGyro.getIntegratedZValue();
//		while ( robot.lrwheel.getCurrentPosition() < Target ) {
//
//			robot.zAccumulated = robot.mrGyro.getIntegratedZValue();
//			telemetry.addData( "freeWheel", robot.freeWheel.getCurrentPosition() );
//			telemetry.addData( "lrwheel", robot.lrwheel.getCurrentPosition() );
//			telemetry.addData( "gyro", robot.zAccumulated );
//			telemetry.update();
//
//
//			robot.rightFront.setPower( -speed - correction / 1000 - ( robot.zAccumulated - target ) / 500 );
//			robot.leftBack.setPower( -speed - correction / 1000 + ( robot.zAccumulated - target ) / 500 );
//			robot.rightBack.setPower( speed - correction / 1000 + ( robot.zAccumulated - target ) / 500 );
//			robot.leftFront.setPower( speed - correction / 1000 - ( robot.zAccumulated - target ) / 500 );
//			//sleep(10);
//			correction = robot.freeWheel.getCurrentPosition() - f;
//
//		}
		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );
	}




	public void driveSideLeftUS( double speed, int distance ) {

		robot.rightFront.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		while ( robot.rangeSensor2.rawUltrasonic() < distance && opModeIsActive() ) {

			robot.rightFront.setPower( -speed );
			robot.leftBack.setPower( -speed );
			robot.rightBack.setPower( speed );
			robot.leftFront.setPower( speed );
			//sleep(10);
//			correction = robot.freeWheel.getCurrentPosition() - f;

		}
		//turn off any extra power
		robot.rightFront.setPower( 0 );
		robot.leftBack.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );
	}




	public void turnAbsolute( int target ) {

		robot.rightFront.setDirection( DcMotor.Direction.FORWARD );
		robot.leftBack.setDirection( DcMotor.Direction.REVERSE );
		robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
		double turnSpeed = 0.3;

		while ( Math.abs( robot.zAccumulated - target ) > 3 && opModeIsActive() ) {  //Continue while the robot direction is further than three degrees from the target
			if ( robot.zAccumulated > target ) {  //if gyro is positive, we will turn right
				robot.rightFront.setPower( turnSpeed );
				robot.leftFront.setPower( -turnSpeed );
				robot.rightBack.setPower( turnSpeed );
				robot.leftBack.setPower( -turnSpeed );      //turn left
			}

			if ( robot.zAccumulated < target ) {  //if gyro is positive, we will turn left
				robot.rightFront.setPower( -turnSpeed );
				robot.leftFront.setPower( turnSpeed );
				robot.rightBack.setPower( -turnSpeed );     //turn right
				robot.leftBack.setPower( turnSpeed );
			}
			telemetry.addData( "Degree", String.format( "%03d", robot.zAccumulated ) );
//			telemetry.update();
			robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
		}

		robot.rightFront.setPower( 0 );
		robot.leftFront.setPower( 0 );
		robot.rightBack.setPower( 0 );     //turn off
		robot.leftBack.setPower( 0 );
	}




	public void turn( int degrees ) {

		robot.rightFront.setDirection( DcMotor.Direction.FORWARD );
		robot.leftBack.setDirection( DcMotor.Direction.REVERSE );
		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		turnAbsolute( degrees );
	}




	public void dontTouch( int target ) {

		robot.rightFront.setDirection( DcMotor.Direction.FORWARD );
		robot.leftBack.setDirection( DcMotor.Direction.REVERSE );
		robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
		double turnSpeed = .3;

		while ( Math.abs( robot.zAccumulated - target ) > 3 && opModeIsActive() ) {  //Continue while the robot direction is further than three degrees from the target
			if ( robot.zAccumulated > target ) {  //if gyro is positive, we will turn right
				robot.rightFront.setPower( turnSpeed );
				robot.leftFront.setPower( -turnSpeed );
				robot.rightBack.setPower( turnSpeed );
				robot.leftBack.setPower( -turnSpeed );      //turn left
				robot.lrwheel.setPower( .5 );
			}

			if ( robot.zAccumulated < target ) {  //if gyro is positive, we will turn left
				robot.rightFront.setPower( -turnSpeed );
				robot.leftFront.setPower( turnSpeed );
				robot.rightBack.setPower( -turnSpeed );     //turn right
				robot.leftBack.setPower( turnSpeed );
				robot.lrwheel.setPower( .5 );
			}
			telemetry.addData( "Degree", String.format( "%03d", robot.zAccumulated ) );
//			telemetry.update();
			robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
		}

		robot.rightFront.setPower( 0 );
		robot.leftFront.setPower( 0 );
		robot.rightBack.setPower( 0 );     //turn off
		robot.leftBack.setPower( 0 );
	}




	public void turnWaffle( int degrees ) {

		robot.rightFront.setDirection( DcMotor.Direction.FORWARD );
		robot.leftBack.setDirection( DcMotor.Direction.REVERSE );
		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		robot.rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		dontTouch( degrees );
	}




	public void driveStraightBackward( double power, int distance ) {

		double leftBSpeed; //Power to feed the motors
		double rightFSpeed;
		double leftFSpeed;
		double rightBSpeed;


		double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
		double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

		robot.rightFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.rightBack.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		robot.leftFront.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		robot.rightFront.setTargetPosition( distance );
		robot.leftBack.setTargetPosition( distance );
		robot.rightBack.setTargetPosition( distance );
		robot.leftFront.setTargetPosition( distance );

		robot.rightFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.rightBack.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		robot.leftFront.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		while ( robot.leftBack.getCurrentPosition() < distance && opModeIsActive() ) {  //While we have not passed out intended distance
			robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

			leftBSpeed = power - ( robot.zAccumulated - target ) / 100;  //Calculate speed for each side
			rightFSpeed = power + ( robot.zAccumulated - target ) / 100;  //See Gyro Straight video for detailed explanation
			leftFSpeed = power - ( robot.zAccumulated - target ) / 100;
			rightBSpeed = power + ( robot.zAccumulated - target ) / 100;  //See Gyro Straight video for detailed explanation

			leftFSpeed = com.qualcomm.robotcore.util.Range.clip( leftFSpeed, -1, 1 );
			rightFSpeed = com.qualcomm.robotcore.util.Range.clip( rightFSpeed, -1, 1 );
			leftBSpeed = com.qualcomm.robotcore.util.Range.clip( leftBSpeed, -1, 1 );
			rightBSpeed = com.qualcomm.robotcore.util.Range.clip( rightBSpeed, -1, 1 );

			robot.leftBack.setPower( leftBSpeed );
			robot.rightFront.setPower( rightFSpeed );
			robot.rightBack.setPower( rightBSpeed );
			robot.leftFront.setPower( leftFSpeed );

			telemetry.addData( "1. Left", robot.leftBack.getPower() );
			telemetry.addData( "2. Right", robot.rightFront.getPower() );
			telemetry.addData( "3. LeftF", robot.leftFront.getPower() );
			telemetry.addData( "4. RightB", robot.rightBack.getPower() );
			telemetry.addData( "Angle: ", robot.zAccumulated );
			telemetry.addData( "3. Distance to go", power + startPosition - robot.leftBack.getCurrentPosition() );
//			telemetry.update();
		}

		robot.leftBack.setPower( 0 );
		robot.rightFront.setPower( 0 );
		robot.rightBack.setPower( 0 );
		robot.leftFront.setPower( 0 );
	}




	//detection pipeline
	static class StageSwitchingPipeline extends OpenCvPipeline {




		Mat yCbCrChan2Mat = new Mat();
		Mat thresholdMat = new Mat();
		Mat all = new Mat();
		List< MatOfPoint > contoursList = new ArrayList<>();




		enum Stage {//color difference. greyscale
			detection,//includes outlines
			THRESHOLD,//b&w
			RAW_IMAGE,//displays raw view
		}




		private Stage stageToRenderToViewport = Stage.detection;
		private Stage[] stages = Stage.values();




		@Override
		public void onViewportTapped() {
			/*
			 * Note that this method is invoked from the UI thread
			 * so whatever we do here, we must do quickly.
			 */

			int currentStageNum = stageToRenderToViewport.ordinal();

			int nextStageNum = currentStageNum + 1;

			if ( nextStageNum >= stages.length ) {
				nextStageNum = 0;
			}

			stageToRenderToViewport = stages[ nextStageNum ];
		}




		@Override
		public Mat processFrame( Mat input ) {

			contoursList.clear();
			/*
			 * This pipeline finds the contours of yellow blobs such as the Gold Mineral
			 * from the Rover Ruckus game.
			 */

			//color diff cb.
			//lower cb = more blue = skystone = white
			//higher cb = less blue = yellow stone = grey
			Imgproc.cvtColor( input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb );//converts rgb to ycrcb
			Core.extractChannel( yCbCrChan2Mat, yCbCrChan2Mat, 2 );//takes cb difference and stores

			//b&w
			Imgproc.threshold( yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV );

			//outline/contour
			Imgproc.findContours( thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE );
			yCbCrChan2Mat.copyTo( all );//copies mat object
			//Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


			//get values from frame
			double[] pixMid = thresholdMat.get( (int) ( input.rows() * midPos[ 1 ] ), (int) ( input.cols() * midPos[ 0 ] ) );//gets value at circle
			valMid = (int) pixMid[ 0 ];

			double[] pixLeft = thresholdMat.get( (int) ( input.rows() * leftPos[ 1 ] ), (int) ( input.cols() * leftPos[ 0 ] ) );//gets value at circle
			valLeft = (int) pixLeft[ 0 ];

			double[] pixRight = thresholdMat.get( (int) ( input.rows() * rightPos[ 1 ] ), (int) ( input.cols() * rightPos[ 0 ] ) );//gets value at circle
			valRight = (int) pixRight[ 0 ];

			//create three points
			Point pointMid = new Point( (int) ( input.cols() * midPos[ 0 ] ), (int) ( input.rows() * midPos[ 1 ] ) );
			Point pointLeft = new Point( (int) ( input.cols() * leftPos[ 0 ] ), (int) ( input.rows() * leftPos[ 1 ] ) );
			Point pointRight = new Point( (int) ( input.cols() * rightPos[ 0 ] ), (int) ( input.rows() * rightPos[ 1 ] ) );

			//draw circles on those points
			Imgproc.circle( all, pointMid, 5, new Scalar( 255, 0, 0 ), 1 );//draws circle
			Imgproc.circle( all, pointLeft, 5, new Scalar( 255, 0, 0 ), 1 );//draws circle
			Imgproc.circle( all, pointRight, 5, new Scalar( 255, 0, 0 ), 1 );//draws circle

			//draw 3 rectangles
			Imgproc.rectangle(//1-3
					all,
					new Point(
							input.cols() * ( leftPos[ 0 ] - rectWidth / 2 ),
							input.rows() * ( leftPos[ 1 ] - rectHeight / 2 ) ),
					new Point(
							input.cols() * ( leftPos[ 0 ] + rectWidth / 2 ),
							input.rows() * ( leftPos[ 1 ] + rectHeight / 2 ) ),
					new Scalar( 0, 255, 0 ), 3 );
			Imgproc.rectangle(//3-5
					all,
					new Point(
							input.cols() * ( midPos[ 0 ] - rectWidth / 2 ),
							input.rows() * ( midPos[ 1 ] - rectHeight / 2 ) ),
					new Point(
							input.cols() * ( midPos[ 0 ] + rectWidth / 2 ),
							input.rows() * ( midPos[ 1 ] + rectHeight / 2 ) ),
					new Scalar( 0, 255, 0 ), 3 );
			Imgproc.rectangle(//5-7
					all,
					new Point(
							input.cols() * ( rightPos[ 0 ] - rectWidth / 2 ),
							input.rows() * ( rightPos[ 1 ] - rectHeight / 2 ) ),
					new Point(
							input.cols() * ( rightPos[ 0 ] + rectWidth / 2 ),
							input.rows() * ( rightPos[ 1 ] + rectHeight / 2 ) ),
					new Scalar( 0, 255, 0 ), 3 );

			switch ( stageToRenderToViewport ) {
				case THRESHOLD: {
					return thresholdMat;
				}

				case detection: {
					return all;
				}

				case RAW_IMAGE: {
					return input;
				}

				default: {
					return input;
				}
			}
		}
//
//
//
	}

//

	}




