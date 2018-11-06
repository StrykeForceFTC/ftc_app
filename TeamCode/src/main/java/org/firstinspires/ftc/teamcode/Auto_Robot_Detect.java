/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This file contains an class that determines which robot
 * we are on based on the configuration.
 *
 */

public class Auto_Robot_Detect
{
    //enum for detection
    public enum teamId
    {
       teamUnknown, team8553, team7228, team15106;
    }

    public teamId TeamId = teamId.teamUnknown;

    Auto_Robot_Detect( HardwareMap hardwareMap )
    {
        WhoAmI( hardwareMap );
    }

    //This looks for the name in the team so, if team is 8553, the name it looks for is 8553.
    //Then it will throw a exception if it was false, which is caught by the catch exception and is made as false in the boolean.
    private boolean AmI( String team, HardwareMap hardwareMap )
    {
        try
        {
            hardwareMap.get(team);
            return true;
        }
        catch ( Exception e)
        {
          return false;
        }

    }


    //This takes the above method and runs it until it returns true.
    //If it returns true, the teamId will change to be given team
    //Ex.  If the 8553 is true, the teamId is set to team8553.
    //If all return false, the team id will stay at default, (team unknown) so that nothing mechanical will break.
    private void WhoAmI( HardwareMap hardwareMap )
    {
        if ( AmI("8553", hardwareMap ))
        {
            TeamId = teamId.team8553;
        }
        else if (AmI("7228", hardwareMap ))
        {
            TeamId = teamId.team7228;
        }
        else if (AmI("15106", hardwareMap ))
        {
            TeamId = teamId.team15106;
        }

    }


}
