#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

int Calib_State(int LimitSwitchPin_Left, int LimitSwitchPin_Right);
float Calib_Process(int LimitSwitchPin_Left, int LimitSwitchPin_Right, float desiredvoltage);

#endif
