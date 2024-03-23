//速度环pid
void SpeedPID_Control()
{
    Speed.Error=(Speed.Set_Speed-Speed.Speed_Car);
    Speed.Integral+=Speed.Error;
    Speed.Integral=constrain_float(Speed.Integral,-300,300);
    Speed.P_Error=Speed.Error;                                          //比例环节
    Speed.I_Error=Speed.Integral;                    //积分环节
    Speed.D_Error=Speed.Error-Speed.L_Error;                            //微分环节

    Speed.Output_PWM=Speed.P*Speed.P_Error+Speed.I*Speed.I_Error+Speed.D*Speed.D_Error;
    //这里为什么有个4000限制啊
    if(Speed.Output_PWM>4000)
        Speed.Output_PWM=4000;
    else if(Speed.Output_PWM<-4000)

        Speed.Output_PWM=-4000;

}
//转向环pd
void TurnPD_Control()
{

    Turn.error=0.9*Turn.Chazhi+0.1*Turn.last_error;
    if(Turn.error<2&&Turn.error>-2)//响应下限
    {
        Turn.error=0;
    }
    Turn.PWM_Dout=Turn.P*Turn.error+Turn.D*(Turn.error-Turn.last_error);
    Turn.last_error = Turn.error;


}