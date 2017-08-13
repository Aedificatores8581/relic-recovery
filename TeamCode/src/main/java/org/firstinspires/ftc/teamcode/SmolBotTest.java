package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Mister Minister Master
 */
@TeleOp(name = "Test Program", group = "anti-bepis")

public class SmolBotTest extends SmolBotTemplate {
    public void start()
    {

    }

    public void loop()
    {
        setLeftPow(2);
        setRightPow(-2);
    }

}
