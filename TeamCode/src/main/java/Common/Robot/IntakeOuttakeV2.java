package Common.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeOuttakeV2 {

    // Four Bar Servos (Control Hub Ports 2 [FourBarLeft], 3 [FourBarRight], 4[FourBarPitch]
    private Servo FourBarLeft;

    private Servo FourBarRight;

    private Servo FourBarPitch;


    // Intake Servos (Control Hub Ports 0 [IntakeLeft], 1 [IntakeRight])
    private CRServo IntakeLeft;

    private CRServo IntakeRight;


    // Horizontal Slides Motor (Expansion Hub Port 0)
    private DcMotorEx HorizontalSlides;


    // Outtake Claw (Expansion Hub Ports 0 [ClawLeft], 1 [ClawRight], 2 [OuttakeArm])
   // private Servo ClawLeft;

    private Servo Claw;

    private Servo OuttakeArm;

    private Servo OuttakeTurret;


    //Vertical Slides Motor ( Expansion Hub Port 1)
    private DcMotorEx VerticalSlides;



    public void initIntake(HardwareMap hwMap){
        FourBarLeft = hwMap.get(Servo.class, "FourBarLeft");
        FourBarRight = hwMap.get(Servo.class, "FourBarRight");
        FourBarPitch = hwMap.get(Servo.class, "FourBarPitch");

        IntakeLeft = hwMap.get(CRServo.class, "IntakeLeft");
        IntakeRight = hwMap.get(CRServo.class, "IntakeRight");

        HorizontalSlides = hwMap.get(DcMotorEx.class, "HorizontalSlides");

        HorizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HorizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HorizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //Intake Sample
    public void intakeIn(){
        IntakeLeft.setPower(1);
        IntakeRight.setPower(-1);
    }
    //Spit out Sample
    public void intakeOut(){
        IntakeLeft.setPower(-.25); //was-.1
        IntakeRight.setPower(.25); //was+.1
    }
    //Turn intake off
    public void intakeOff(){
        IntakeLeft.setPower(0);
        IntakeRight.setPower(0);
    }
    //Lower four bar to intake sample
    public void fourBarDown(){
        FourBarLeft.setPosition(0);//was.09
        FourBarRight.setPosition(1);//was.47
    }
    //Extend four bar to search for sample
    public void fourBarSearch(){
        FourBarLeft.setPosition(.1+.05);//was .19
        FourBarRight.setPosition(.9-.05);//was .37
    }

    public void fourBarTransfer(){
        FourBarLeft.setPosition(.2+.025-.05);
        FourBarRight.setPosition(.8-.025+.05);
    }

    //Retract four bar
    public void fourBarStowed(){
        FourBarLeft.setPosition(.26);//was .35
        FourBarRight.setPosition(.74);//was .2
    }


    public void fourBarPitchSearch(){
        FourBarPitch.setPosition(.2-.03);
    }

    public void fourBarPitchStowed(){
        FourBarPitch.setPosition(.65);
    }
    public void fourBarPitchTransfer(){
        FourBarPitch.setPosition(.75+.075);
    }
    public void fourBarPitchStart (){
        FourBarPitch.setPosition(.7);
    }

    public void horizontalSlidesHome(){
        HorizontalSlides.setTargetPosition(0);
        HorizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HorizontalSlides.setPower(.35);
    }

    public int horizontalSlidesReturnPosition(){
        return HorizontalSlides.getCurrentPosition();
    }
    public void horizontalSlidesManualControl(double speed){
        HorizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HorizontalSlides.setPower(speed);
    }

    public void recalibrateHorizontalSlide(){
        HorizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initOuttake(HardwareMap hwMap){
       // ClawLeft = hwMap.get(Servo.class, "ClawLeft");
        Claw = hwMap.get(Servo.class, "Claw");
        OuttakeArm = hwMap.get(Servo.class, "OuttakeArm");
        OuttakeTurret = hwMap.get(Servo.class, "OuttakeTurret");

        VerticalSlides = hwMap.get(DcMotorEx.class, "VerticalSlides");

        VerticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VerticalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

   /* public void armTransfer(){
        OuttakeArm.setPosition(1);
    }

    public void armPreScore(){
        OuttakeArm.setPosition(.25);
    }

    public void armPickUp(){
        OuttakeArm.setPosition(.1125);
    }

    public void armStowed(){
        OuttakeArm.setPosition(.05);
    }
*/
    public void armTurretForward(){
        OuttakeTurret.setPosition(1-.05);
    }

    public void armTurretBackward(){
        OuttakeTurret.setPosition(0);
    }

    public void armTransfer (){
        //this will change based on how it gets attached
        OuttakeArm.setPosition(.95);
    }

    /*public void armScore(){
        OuttakeArm.setPosition(.75);
    }*/

    public void armHorizontal (){
        //for pick up on wall either side
        //higher number = further up
        OuttakeArm.setPosition(.7-.05);
    }

    public void armPickup (){
        //for pick up on wall either side
        //higher number = further up
        OuttakeArm.setPosition(.7);
    }

    public void armChamber (){
        //45 degree to place at chamber
        OuttakeArm.setPosition(.55-.15);
    }

    public void armStowed (){
        OuttakeArm.setPosition(.85);
    }

    public void clawOpen(){

        Claw.setPosition(.76);  //tune this
    }

    public void clawClosed(){

        Claw.setPosition(.4); //tune this
    }

    public void liftStowed(){
        VerticalSlides.setTargetPosition(0);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.35);
    }


    public void liftSetHighChamber(){
        VerticalSlides.setTargetPosition(-600);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.35);
    }
    public void liftTransfer(){
        VerticalSlides.setTargetPosition(-450);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftLowChamber(){
        VerticalSlides.setTargetPosition(-400);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftLowBasket(){
        VerticalSlides.setTargetPosition(-600);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftHighChamber(){
        VerticalSlides.setTargetPosition(-800);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }
    public void liftAutoHighChamber(){
        VerticalSlides.setTargetPosition(-825);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftAutoHighBasket(){
        VerticalSlides.setTargetPosition(-3000);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftHighBasket(){
        VerticalSlides.setTargetPosition(-2800);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftSwing(){
        VerticalSlides.setTargetPosition(-200-150);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftPreTransfer(){
        VerticalSlides.setTargetPosition(-600);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }
    public void liftManualControl(double speed){
        VerticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VerticalSlides.setPower(speed);
    }

    public void recalibrateLift(){
        VerticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean liftHasArrived() {
        if (Math.abs(VerticalSlides.getTargetPosition() - VerticalSlides.getCurrentPosition()) < 50) {

            return true;

        } else {
            return false;
        }

    }
}



