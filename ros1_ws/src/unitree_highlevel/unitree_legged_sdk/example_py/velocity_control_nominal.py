#!/usr/bin/python

import sys
import time
import math

sys.path.append("../lib/python/amd64")
import robot_interface as sdk


if __name__ == "__main__":
    d = {
        "FR_0": 0,
        "FR_1": 1,
        "FR_2": 2,
        "FL_0": 3,
        "FL_1": 4,
        "FL_2": 5,
        "RR_0": 6,
        "RR_1": 7,
        "RR_2": 8,
        "RL_0": 9,
        "RL_1": 10,
        "RL_2": 11,
    }
    PosStopF = math.pow(10, 9)
    VelStopF = 16000.0
    HIGHLEVEL = 0xEE
    LOWLEVEL = 0xFF

    # udp = sdk.UDP(8080, "192.168.123.10", 8007, 614, 807, False, sdk.RecvEnum.nonBlock)
    udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
    safe = sdk.Safety(sdk.LeggedType.B1)

    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)

    Tpi = 0
    motiontime = 0

    while True:
        time.sleep(0.002)
        motiontime += 1

        udp.Recv()
        udp.GetRecv(state)

        speed_fr0 = 0 - state.motorState[d["FR_0"]].q
        speed_fl0 = 0 - state.motorState[d["FL_0"]].q
        speed_rr0 = 0 - state.motorState[d["RR_0"]].q
        speed_rl0 = 0 - state.motorState[d["RL_0"]].q

        cmd.motorCmd[d["FR_0"]].q = 0.0
        cmd.motorCmd[d["FR_0"]].dq = speed_fr0
        cmd.motorCmd[d["FR_0"]].Kp = 10
        cmd.motorCmd[d["FR_0"]].Kd = 1
        cmd.motorCmd[d["FR_0"]].tau = -5.0

        cmd.motorCmd[d["FL_0"]].q = 0
        cmd.motorCmd[d["FL_0"]].dq = speed_fl0
        cmd.motorCmd[d["FL_0"]].Kp = 10
        cmd.motorCmd[d["FL_0"]].Kd = 1
        cmd.motorCmd[d["FL_0"]].tau = 5.0

        cmd.motorCmd[d["RR_0"]].q = 0
        cmd.motorCmd[d["RR_0"]].dq = speed_rr0
        cmd.motorCmd[d["RR_0"]].Kp = 10
        cmd.motorCmd[d["RR_0"]].Kd = 1
        cmd.motorCmd[d["RR_0"]].tau = -5.0

        cmd.motorCmd[d["RL_0"]].q = 0
        cmd.motorCmd[d["RL_0"]].dq = speed_rl0
        cmd.motorCmd[d["RL_0"]].Kp = 10
        cmd.motorCmd[d["RL_0"]].Kd = 1
        cmd.motorCmd[d["RL_0"]].tau = 5.0

        if motiontime >= 500:
            speed_fr1 = 0.9 - state.motorState[d["FR_1"]].q
            speed_fl1 = 0.9 - state.motorState[d["FL_1"]].q
            speed_rr1 = 0.9 - state.motorState[d["RR_1"]].q
            speed_rl1 = 0.9 - state.motorState[d["RL_1"]].q

            speed_fr2 = -1.8 - state.motorState[d["FR_2"]].q
            speed_fl2 = -1.8 - state.motorState[d["FL_2"]].q
            speed_rr2 = -1.8 - state.motorState[d["RR_2"]].q
            speed_rl2 = -1.8 - state.motorState[d["RL_2"]].q

            cmd.motorCmd[d["FR_1"]].q = 0.9
            cmd.motorCmd[d["FR_1"]].dq = speed_fr1
            cmd.motorCmd[d["FR_1"]].Kp = 20
            cmd.motorCmd[d["FR_1"]].Kd = 2
            cmd.motorCmd[d["FR_1"]].tau = 0.0

            cmd.motorCmd[d["FL_1"]].q = 0.9
            cmd.motorCmd[d["FL_1"]].dq = speed_fl1
            cmd.motorCmd[d["FL_1"]].Kp = 20
            cmd.motorCmd[d["FL_1"]].Kd = 2
            cmd.motorCmd[d["FL_1"]].tau = 0.0

            cmd.motorCmd[d["RR_1"]].q = 0.9
            cmd.motorCmd[d["RR_1"]].dq = speed_rr1
            cmd.motorCmd[d["RR_1"]].Kp = 20
            cmd.motorCmd[d["RR_1"]].Kd = 2
            cmd.motorCmd[d["RR_1"]].tau = 0.0

            cmd.motorCmd[d["RL_1"]].q = 0.9
            cmd.motorCmd[d["RL_1"]].dq = speed_rl1
            cmd.motorCmd[d["RL_1"]].Kp = 20
            cmd.motorCmd[d["RL_1"]].Kd = 2
            cmd.motorCmd[d["RL_1"]].tau = 0.0

            cmd.motorCmd[d["FR_2"]].q = -1.8
            cmd.motorCmd[d["FR_2"]].dq = speed_fr2
            cmd.motorCmd[d["FR_2"]].Kp = 30
            cmd.motorCmd[d["FR_2"]].Kd = 3
            cmd.motorCmd[d["FR_2"]].tau = 0.0

            cmd.motorCmd[d["FL_2"]].q = -1.8
            cmd.motorCmd[d["FL_2"]].dq = speed_fl2
            cmd.motorCmd[d["FL_2"]].Kp = 30
            cmd.motorCmd[d["FL_2"]].Kd = 3
            cmd.motorCmd[d["FL_2"]].tau = 0.0

            cmd.motorCmd[d["RR_2"]].q = -1.8
            cmd.motorCmd[d["RR_2"]].dq = speed_rr2
            cmd.motorCmd[d["RR_2"]].Kp = 30
            cmd.motorCmd[d["RR_2"]].Kd = 3
            cmd.motorCmd[d["RR_2"]].tau = 0.0

            cmd.motorCmd[d["RL_2"]].q = -1.8
            cmd.motorCmd[d["RL_2"]].dq = speed_rl2
            cmd.motorCmd[d["RL_2"]].Kp = 30
            cmd.motorCmd[d["RL_2"]].Kd = 3
            cmd.motorCmd[d["RL_2"]].tau = 0.0

            Tpi += 1

        # if(motiontime > 10):
        #     safe.PowerProtect(cmd, state, 1)

        udp.SetSend(cmd)
        udp.Send()
