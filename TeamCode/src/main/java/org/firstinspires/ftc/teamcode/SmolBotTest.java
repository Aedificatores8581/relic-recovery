package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Mister Minister Master
 */
@TeleOp(name = "Test Program", group = "anti-bepis")

public class SmolBotTest extends TestBotTemplate {
    private double power = 1;
    public void start(){}

    @Override
    public void loop()
    {
        setLeftPow(-power);
        setRightPow(power);
    }

}
