/**
 * @file autotune_pid.c
 * @brief Relay-feedback (bang-bang) based PID autotuning for position loop.
 * @authot Enes UNLU
 * 
 * The algorithm applies +/-h excitation around refDeg, measures oscillation
 * amplitude (A) and period (T), computes ultimate gain Kc = 4h/(pi*A),
 * then derives PID gains using mode-dependent filter coefficients (tuningTable).
 *
 * Timing: sampling is driven by g_autoTunePIDTimer.Flag (expected fixed period).
 * Units: degrees for refDeg/AcDeg/Error, milliseconds for Counter*TICK, etc.
 */

void ResetAutoPidParams(AutoPIDParam *atPID ){
   atPID->Kp             =  0.0000f;
   atPID->Kd             =  0.0000f;
   atPID->Ki             =  0.0000f;
   atPID->ErrVal         =  0.0000f;
   atPID->CaseState      =  0;
   atPID->h              =  5000.0;
   atPID->TempErrorMin   =  9000.0;
   atPID->TempErrorMax   =  -9000.0;
   atPID->preTempError   =  0;
   atPID->tempError      =  0;
   atPID->T              =  0;
   atPID->Counter        =  0;
   atPID->Kc             =  0.0f;
   atPID->StartAtPID     =  0;
   atPID->peakCount      =  0;
   atPID->lastPeakTime   =  0;
   atPID->rising         =  0;
   atPID->mode           =  0;                         //default mod
}

// Case 0: Initialization & move to refDeg.
// Transition: when |ErrVal| < 0.3 deg -> Case 1 (start relay test)

// Case 1: Relay test (apply +/-h). Sample error each timer tick.
// Detect peaks to estimate period T and amplitude A.
// Transition: after N samples (Counter>=150) -> Case 2

// Case 2: Compute Kc, T_sec, then Kp/Ki/Kd using tuningTable[mode].
// Transition: -> Case 3

// Case 3: Publish tuned gains to global PID variables and stop autotune.

void AutoTuneHandler(AutoPIDParam *atPID){
    switch(atPID->CaseState){
        case 0:
            ResetAutoPidParams(atPID);
            
            GoToPosPIDParam((atPID->refDeg), 1000);                 // Go to reference degree
            
            GainFilter currentFilter = tuningTable[atPID->mode];    // Mode configuration
            
            float AcDeg = GetFDegData();                            // Instance degree data
            atPID->ErrVal = (AcDeg - atPID->refDeg);
            
            if(atPID->ErrVal < 0.3 && atPID->ErrVal > -0.3){        // If we are at reference degree
                atPID->StartAtPID = 1;
                atPID->CaseState  = 1;
            }
        break;

        case 1:   
            if(atPID->StartAtPID == 1){
                atPID->StartAtPID = 0;  
                SetTimer3State(TIMER_OPEN); 
            }
            AcDeg = GetFDegData();  
            atPID->ErrVal = (AcDeg - atPID->refDeg);                // Error degree calc

            if(atPID->ErrVal > 0)
                SendMovePacket( ((-1) * (atPID->h)));               // Tuning 
            else
                SendMovePacket(  atPID->h);


            if(g_autoTunePIDTimer.Flag == 1){                       // Sampling and calculations of parameters
                g_autoTunePIDTimer.Flag = 0;
                atPID->Counter++;    
                   
                float tempError = atPID->ErrVal;
                
                if(tempError > atPID->TempErrorMax) 
                atPID->TempErrorMax = tempError;
                
                if(tempError < atPID->TempErrorMin)
                atPID->TempErrorMin = tempError;
                
                if(tempError > atPID->preTempError)
                atPID->rising = 1;
                           
                if((tempError < atPID->preTempError) && (atPID->rising == 1)){
                atPID->rising = 0;
                atPID->peakCount++;

                if(atPID->peakCount == 1 && atPID->lastPeakTime == 0)
                atPID->lastPeakTime = (uint32_t)atPID->Counter;
            
                else if(atPID->peakCount == 4)
                atPID->T = (uint32_t)((atPID->Counter - atPID->lastPeakTime) / 3);
            
                }
                atPID->preTempError = tempError;
                
            }
            if(atPID->Counter >= 150)              // 15 second
                atPID->CaseState = 2;

#if   DEBUG_AT_PID_CONTROL_1
        fprintf(DEBUG_CONT_PORT_NAME,"\n ========================== \n");
        fprintf(DEBUG_CONT_PORT_NAME,"lastPeakTime %u",atPID->lastPeakTime);
        fprintf(DEBUG_CONT_PORT_NAME,"Counter %u",atPID->Counter);
        fprintf(DEBUG_CONT_PORT_NAME,"T %u",atPID->T);
        fprintf(DEBUG_CONT_PORT_NAME,"TempSpeedMax %.4f",atPID->TempErrorMax);
        fprintf(DEBUG_CONT_PORT_NAME,"TempSpeedMin %.4f",atPID->TempErrorMin);
        fprintf(DEBUG_CONT_PORT_NAME,"\n ========================== \n");
#endif     
        break;

        case 2:  
            SetTimer3State(TIMER_STOP);
            float A = (atPID->TempErrorMax - atPID->TempErrorMin) * 0.5f;    
            atPID->Kc = 4 * atPID->h / (pi * A);

            float T_sec = (atPID->T * CONT_AT_PID_TILT_CHUNK) / 1000.0f;     // T › 100 ms sample time

            atPID->Kp = (currentFilter.KpFilter)  * atPID->Kc;
            atPID->Ti = (currentFilter.KiFilter)  * T_sec;
            atPID->Td = (currentFilter.KdFilter)  * T_sec;
            
            atPID->Ki = atPID->Kp / atPID->Ti;
            atPID->Kd = atPID->Kp * atPID->Td;
            
            atPID->CaseState = 3;
            
#if   DEBUG_AT_PID_CONTROL_2
        fprintf(DEBUG_CONT_PORT_NAME,"\n ========================== \n");
        fprintf(DEBUG_CONT_PORT_NAME,"Kc %.4f",atPID->Kc);
        fprintf(DEBUG_CONT_PORT_NAME,"Amplitude %.4f",A);
        fprintf(DEBUG_CONT_PORT_NAME,"T_sec %.4f",T_sec);
        fprintf(DEBUG_CONT_PORT_NAME,"atPID->T %.4f",atPID->T);
#endif     
        break;
        
        case 3: 
            Kp = atPID->Kp;
            Ki = atPID->Ki;
            Kd = atPID->Kd;
            g_startAutoTune = 0;
#if   DEBUG_AT_PID_CONTROL_3
        fprintf(DEBUG_CONT_PORT_NAME,"atPID->Kp %.4f",atPID->Kp);
        fprintf(DEBUG_CONT_PORT_NAME,"atPID->Ki %.4f",atPID->Ki);
        fprintf(DEBUG_CONT_PORT_NAME,"atPID->Kd %.4f",atPID->Kd);
        fprintf(DEBUG_CONT_PORT_NAME,"\n ========================== \n");
#endif     
        break;
    }
}


