#define TRUE 1
#define FALSE 0

int upperbound = 0; 
int lowerbound = 0; 
int LS_CNT_L = 0, LS_CNT_R = 0;



float Calib_Process(int LimitSwitchPin_Left, int LimitSwitchPin_Right, float desiredvoltage)
{
    int voltage_return = 0; 
    // int LS_L_CNT = 0, LS_R_CNT = 0;
    if((digitalRead(LimitSwitchPin_Left) == HIGH) || (digitalRead(LimitSwitchPin_Right) == HIGH))
    {
        voltage_return = -desiredvoltage;
    }
    return voltage_return; 
}

int Calib_State(int LimitSwitchPin_Left, int LimitSwitchPin_Right)
/*State Judgement for whether exiting the CALIBRATION state*/
{
    
    if(digitalRead(LimitSwitchPin_Left) == HIGH)
    {
        LS_CNT_L++;
    }
    if(digitalRead(LimitSwitchPin_Right) == HIGH)
    {
        LS_CNT_R++;
    }
    //
    if(LS_CNT_L && LS_CNT_R)
    {
        return TRUE; 
    }
    else
    {
        return FALSE;
    }
}