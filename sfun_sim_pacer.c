/* 
 *  Slow down a simulation to run at approximately real-time (or a chosen
 *  ratio of real-time).
 */
#define S_FUNCTION_NAME sfun_sim_pacer
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#ifdef _WIN32
/* On Windows we are using multimedia and performance timers/counters */
#include <windows.h>
#define clock_t int64_t
#else
/* Everything else can use time and usleep */
#include <time.h>
#include <unistd.h>
#endif

#ifdef MATLAB_MEX_FILE
#include "matrix.h"
#endif

/* Undefined value for timer resolution */
#define DEF_NO_RES 1e6

/* I (integer) work storage */
enum { NUM_I_WORKS };

/* P (pointer) work storage */
enum { PW_CLOCK_LAST = 0, NUM_P_WORKS };

/* R (real) work storage */
enum { RW_T_FRAC = 0, RW_SIMT_LAST, RW_ACCUMERR, RW_TIMER_RES, NUM_R_WORKS };

/* S-function parameters*/
enum { SP_T_FRAC = 0, SP_ENABLE_OUT, NUM_S_PARAMS };
#define PARAM_T_FRAC ssGetSFcnParam( S, SP_T_FRAC )
#define PARAM_ENABLE_OUT ssGetSFcnParam( S, SP_ENABLE_OUT )

/* Input port definition */
enum { NUM_INPUT_PORTS = 0 };

/* Output port definition */
enum { OP_TERR = 0, NUM_OUTPUT_PORTS };

/**
 * Sanity check the input parameters.
 */
#ifdef MATLAB_MEX_FILE
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters( SimStruct *S ) {
    
    /* Check first input (time fraction) */
    if( !mxIsDouble( PARAM_T_FRAC ) || mxIsComplex( PARAM_T_FRAC ) || !mxIsScalar( PARAM_T_FRAC ) || mxGetScalar( PARAM_T_FRAC )<=0 ) {
        ssSetErrorStatus( S, "Parameter t_frac must be a positive definite real, scalar double." );
        return;
    }
    
    /* Check second input (flag to enable the outputs) */
    if( !mxIsDouble( PARAM_ENABLE_OUT ) || mxIsComplex( PARAM_ENABLE_OUT ) || !mxIsScalar( PARAM_ENABLE_OUT ) ) {
    }
}
#endif

/**
 * Initialise all of the model sizes 
 */
static void mdlInitializeSizes( SimStruct *S ) {
    
    int_T i;
    
    ssSetNumSFcnParams( S, NUM_S_PARAMS );
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Parameter mismatch will be reported by Simulink */
        return;
    }
#ifdef MATLAB_MEX_FILE
    else {
		mdlCheckParameters(S);
		if (ssGetErrorStatus(S) != NULL)
		    return;
	}
#endif

    ssSetNumContStates( S, 0 );
    ssSetNumDiscStates( S, 0 );

    ssSetNumRWork( S, NUM_R_WORKS );
    ssSetNumIWork( S, NUM_I_WORKS );
    ssSetNumPWork( S, NUM_P_WORKS );
    
    if (!ssSetNumInputPorts( S, NUM_INPUT_PORTS )) return;    

    if ( mxGetScalar( PARAM_ENABLE_OUT ) != 0 ) {
        if ( !ssSetNumOutputPorts( S, NUM_OUTPUT_PORTS ) ) return;
        ssSetOutputPortWidth( S, OP_TERR, 1 );
        ssSetOutputPortDataType( S, OP_TERR, SS_DOUBLE );
    }else{
        if (!ssSetNumOutputPorts( S, 0 )) return;
    }
    
    ssSetSimStateCompliance( S, USE_DEFAULT_SIM_STATE );
    
    /* No parameters are tunable */
    for ( i=0; i<NUM_S_PARAMS; i++ ) 
        ssSetSFcnParamTunable( S, i, SS_PRM_NOT_TUNABLE );

    ssSetOptions( S,
            SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME
            | SS_OPTION_DISCRETE_VALUED_OUTPUT 
            | SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE );
}

/**
 * Inialise the sample times - all inherited
 */
static void mdlInitializeSampleTimes( SimStruct *S ) {
    ssSetSampleTime( S, 0, INHERITED_SAMPLE_TIME );
    ssSetOffsetTime( S, 0, FIXED_IN_MINOR_STEP_OFFSET );
    ssSetModelReferenceSampleTimeInheritanceRule( S, USE_DEFAULT_FOR_DISCRETE_INHERITANCE );
}

/**
 * Model start function - allocate storage for pointers and save current time
 */
#define MDL_START
static void mdlStart( SimStruct *S ) {
#ifdef _WIN32
    TIMECAPS tc;
    int_T wTimerRes;
#endif
    real_T timerRes;
    
    /* Allocate storage, and store the current time */
    clock_t *startTime = malloc( sizeof( clock_t ) );
#ifdef _WIN32
    QueryPerformanceCounter( startTime );
#else
    *startTime = clock();
#endif
    ssSetPWorkValue( S, PW_CLOCK_LAST, (void *)startTime );
    
    /* Store the relative time parameter, zero error, and last time */
    ssSetRWorkValue( S, RW_T_FRAC, mxGetScalar( PARAM_T_FRAC ) );
    ssSetRWorkValue( S, RW_SIMT_LAST, 0 );
    ssSetRWorkValue( S, RW_ACCUMERR, 0 );
    
    timerRes = DEF_NO_RES;
    /* On Windows, set the timer resolution to its minimum */
#ifdef _WIN32
    if( timeGetDevCaps( &tc, sizeof(TIMECAPS) ) == TIMERR_NOERROR ) {
        wTimerRes = min( max( tc.wPeriodMix, 1 ), tc.wPeriodMax );
        timeBeginPeriod(wTimerRes);
        timerRes = wTimerRes / 1000.0;
    }
#else
    /* On other systems, time.h should give about 1e-5 accuracy */
    timerRes = 1e-6;
#endif
    ssSetRWorkValue( S, RW_TIMER_RES, timerRes );
}

/*
 * Main model work function. Use sleep when possible, then busy wait the
 * the last small amounts.
 */
static void mdlOutputs( SimStruct *S, int_T tid ) {
    /* Real time values */
    int64_t *t_last = (int64_t *)ssGetPWorkValue( S, PW_CLOCK_LAST );
    int64_t t_now, t_target;
#ifdef _WIN32
    clock_t CLOCKS_PER_SEC;
    QueryPerformanceFrequency( &CLOCKS_PER_SEC );
#endif
    
    /* Sim-time values */
    time_T s_last = (time_T) ssGetRWorkValue( S, RW_SIMT_LAST );
    time_T s_now = ssGetT( S );
    time_T s_elapsed = ( s_now - s_last ) / (time_T)ssGetRWorkValue( S, RW_T_FRAC );
    time_T delay, error, timeeps;
    
    /* Calculate the target clock value */
    t_target = *t_last + (clock_t)( s_elapsed * CLOCKS_PER_SEC );
    
    /* Feedback value to correct for non-zero mean in task execution times */
    timeeps = -0.5*ssGetRWorkValue( S, RW_ACCUMERR );
    
    /* Sleep first so we aren't hogging to CPU */
    time_T t_res = ((time_T)ssGetRWorkValue( S, RW_TIMER_RES ));
    
#ifdef _WIN32
    /* On windows, use Sleep with multimedia timers (configured in mdlStart) */
    QueryPerformanceCounter( &t_now );
    delay = ((time_T)( t_target - t_now ))/CLOCKS_PER_SEC - timeeps;
    if ( delay > 2*t_res ) {
        Sleep( (unsigned int) (( delay - t_res )*1000) );
    }
#else
    /* Other systems, use usleep */
    t_now = clock();
    delay = ((time_T)( t_target - t_now ))/CLOCKS_PER_SEC - timeeps;
    if ( delay > 2*t_res ) {
        usleep( (unsigned int) (( delay - t_res )*1e6 ) ); 
    }
#endif
    
    /* Busy-wait for times smaller than the timer resolution */
    do {
#ifdef _WIN32
        QueryPerformanceCounter( &t_now );
#else
        t_now = clock();
#endif
        delay = ((time_T)( t_target - t_now ))/CLOCKS_PER_SEC;        
    } while ( delay > timeeps );
    
    /* Update outputs, and save the previous values. Error is in real-time seconds */
    error = (real_T)( s_elapsed - ((time_T)( t_now - *t_last ) / CLOCKS_PER_SEC ) );
    ssSetRWorkValue( S, RW_ACCUMERR, ssGetRWorkValue( S, RW_ACCUMERR ) + error*(s_now-s_last)  );
    if ( ssGetNumOutputPorts(S)>0 ) {
        *ssGetOutputPortRealSignal( S, OP_TERR ) = error;
    }
    ssSetRWorkValue( S, RW_SIMT_LAST, s_now );
    *t_last = t_target;
}

/*
 * Store values when pausing/resuming the simulation
 */
#if defined(MATLAB_MEX_FILE) 
#define MDL_SIM_STATUS_CHANGE
static void mdlSimStatusChange( SimStruct *S, ssSimStatusChangeType simStatus ) { 
    clock_t *last = (clock_t *) ssGetPWorkValue( S, PW_CLOCK_LAST );
    clock_t now;
#ifdef _WIN32
    QueryPerformanceCounter( &now );
#else
    now = clock();
#endif
    switch( simStatus ) {
        case SIM_PAUSE:
            /* Store current elapsed time in the clock time. */
            *last = now - *last;
            break;
        case SIM_CONTINUE:
            /* Subtract previously elapsed time from current time to resume */
            *last = now - *last;
            break;
    }
} 
#endif

/*
 * Model terminate function - free pointer storage and stop the multimedia
 * timer on Windows if we started it.
 */
static void mdlTerminate( SimStruct *S ) {
  int i;
  for (i = 0; i<ssGetNumPWork(S); i++) {
    if (ssGetPWorkValue(S,i) != NULL) {
      free(ssGetPWorkValue(S,i));
    }
  }
  
/* On Windows, end the timer (if we started it) */
#ifdef _WIN32
  if( ssGetRWorkValue( S, RW_TIMER_RES )<DEF_NO_RES ) {
      timerEndPeriod( (unsigned int)ssGetRWorkValue( S, RW_TIMER_RES )*1000 );
      ssSetRWorkValue( S, RW_TIMER_RES, DEF_NO_RES );
  }
#endif
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif