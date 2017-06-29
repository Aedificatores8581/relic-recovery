package org.firstinspires.ftc.teamcode;

//------------------------------------------------------------------------------
//
// PootisBotManual
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * I made this from scratch.
 *
 * @author Pootis Man
 */
@TeleOp(name = "RevBot: Tele-Op", group = "bepis")
public class SmolBotSquare extends SmolBotTemplate
{
    State state;


    //--------------------------------------------------------------------------
    //
    //
    //
    //--------
    // Constructs the class.
    //
    // The system calls this member when the class is instantiated.
    //--------
    public SmolBotSquare() {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    }

    @Override
    public void start() {
        setLeftPow(0.5);
        setRightPow(0.5);
        state = State.MOVING;
    }

    private enum State {
        MOVING,
        TURNING
    }

    //--------------------------------------------------------------------------
    //
    // loop
    //
    //-------
    // Initializes the class.
    //
    // The system calls this member repeatedly while the OpMode is running.
    //--------
    @Override public void loop () {
        switch (state) {
            case MOVING:
                if (checkEncoder(500)) {
                    setLeftPow(-0.5);
                    setRightPow(0.5);
                    state = State.TURNING;
                }
            case TURNING:
                if (checkEncoder(200)) {
                    setLeftPow(0.5);
                    setRightPow(0.5);
                    state = State.TURNING;
                }
        }
    }

} // PootisBotManual
