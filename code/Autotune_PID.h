typedef struct tagGainFilter{
   float KpFilter;
   float KiFilter;
   float KdFilter;
}GainFilter;

const GainFilter tuningTable[] = {
    {0.60f, 0.50f, 0.125f}, // Index 0: Klasik ZN
    {0.45f, 2.20f, 0.159f}, // Index 1: Tyreus-Luyben
    {0.20f, 0.50f, 0.333f}  // Index 2: No Overshoot
};

typedef struct tagAutoPIDParam{
   float    Kp;
   float    Kd;
   float    Ki;
   float    Td;
   float    Ti;
   float    Kc;
   float    ErrVal;
   float    refDeg;
   float    h;
   float    tempError;
   float    TempErrorMin;
   float    TempErrorMax;
   float    preTempError;
   float    T;
   uint32_t lastPeakTime;
   uint8_t  StartAtPID;
   uint8_t  CaseState;
   uint8_t  Counter;
   uint8_t  rising;
   uint8_t  peakCount;
   uint8_t  mode;
}AutoPIDParam;
AutoPIDParam atPID;
