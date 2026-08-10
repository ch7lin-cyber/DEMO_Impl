#include "DL_DEMO_Impl.h"
#include <stdint.h>

using namespace IEC;


//CK_ERROR set_auto_tune_param(CK_AUTO_TUNE autotune);
//CK_ERROR set_controller_param(CK_CONTROLLER controller, float step_time);

/*
void print_applied_parameters(CK_CONTROLLER controller) {
    uint32_t pb = 0;
    uint32_t ti = 0;
    uint32_t td = 0;
    uint32_t ipreload = 0;
    ckControllerGetUIntParam(controller, DT_PID_PARAM_PB, &pb);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_TI, &ti);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_TD, &td);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_IPRELOAD, &ipreload);
}
*/
/* 
CK_ERROR set_auto_tune_param(CK_AUTO_TUNE autotune) {
    (ckAutoTuneSetIntParam(autotune, DT_AUTO_TUNE_PARAM_SV, 1000));

    (ckAutoTuneSetEnumParam(autotune, DT_AUTO_TUNE_PARAM_CONTROL_ACTION, CK_CONTROL_ACTION_HEAT));

    (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_MV1_LIMIT_HIGH, 1000));
    (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_MV2_LIMIT_HIGH, 1000));
    (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_SV_RATIO, 100));
    (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_PB_RATIO, 100));
    (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_TI_RATIO, 100));

    return CK_NO_ERR;
}
*/

CK_ERROR get_pid_param(CK_CONTROLLER controller, uint32_t * pb_,
                             uint32_t * ti_, uint32_t * td_, uint32_t * alpha_, uint32_t * beta_, uint32_t * gamma_, uint32_t * ipreload_)
{

    ckControllerGetUIntParam(controller, DT_PID_PARAM_PB, pb_);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_TI, ti_);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_TD, td_);


    ckControllerGetUIntParam(controller, DT_PID_PARAM_ALPHA, alpha_);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_BETA, beta_);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_GAMMA, gamma_);
    ckControllerGetUIntParam(controller, DT_PID_PARAM_IPRELOAD, ipreload_);

    return CK_NO_ERR;
}

/*
CK_ERROR set_pid_param(CK_CONTROLLER controller, uint32_t * pb_,
                             uint32_t * ti_, uint32_t * td_)
{
    
    ckControllerSetUIntParam(controller, DT_PID_PARAM_PB, pb_);
    ckControllerSetUIntParam(controller, DT_PID_PARAM_TI, ti_);
    ckControllerSetUIntParam(controller, DT_PID_PARAM_TD, td_);

    return CK_NO_ERR;
}

*/
/*
CK_ERROR set_controller_param(CK_CONTROLLER controller, float step_time) {
    (ckControllerSetEnumParam(controller, DT_PID_PARAM_PID_MODE, 0));
    (ckControllerSetEnumParam(controller, DT_PID_PARAM_CONTROL_ACTION, CK_CONTROL_ACTION_HEAT));

    (ckControllerSetBoolParam(controller, DT_PID_PARAM_DIS_COMP_SWITCH, false));

    (ckControllerSetIntParam(controller, DT_PID_PARAM_SV, 1000));
    (ckControllerSetIntParam(controller, DT_PID_PARAM_DEADBAND, 0));
    (ckControllerSetIntParam(controller, DT_PID_PARAM_D_LIMIT_GATE, 300));
    (ckControllerSetIntParam(controller, DT_PID_PARAM_STABLE_RANGE, 0));

    (ckControllerSetUIntParam(controller, DT_PID_PARAM_MV1_LIMIT_HIGH, 1000));
    (ckControllerSetUIntParam(controller, DT_PID_PARAM_MV1_LIMIT_LOW, 0));
    (ckControllerSetUIntParam(controller, DT_PID_PARAM_MV2_LIMIT_HIGH, 1000));
    (ckControllerSetUIntParam(controller, DT_PID_PARAM_MV2_LIMIT_LOW, 0));
    (ckControllerSetUIntParam(controller, DT_PID_PARAM_PDOF, 0));

    const int32_t period = (int32_t)(step_time * 100.0F);  // unit: 10ms // 0.05 * 100.0 = 5 
    (ckControllerSetUIntParam(controller, DT_PID_PARAM_PERIOD, period));

    return CK_NO_ERR;
}
*/
//----------------------------------------------------------------------------//

// FUNCTION_BLOCK FB_PID_CONTROL
void IEC::DL_SSM_CTRL_LIB::FB_PID_CONTROL::__Init()
{
	PID_controlAction = CK_CONTROL_ACTION_HEAT;
	PID_mode = 0;
	SV = 0;
	PV = 0;
    MV1 = 0;
    MV2 = 0;
	PID_deadBand = 0;
	PID_D_limitGate = 300;
	PID_stableRange = 0;
	PID_err = 0;
    PID_sv2DOF = 0;
    PID_I_preload = 0;
	PID_PB = 100;
	PID_TI = 100;
	PID_TD = 100;
	PID_cPB = 0;
	PID_cTI = 0;
	PID_cTD = 0;
	PID_mv1_LimitHigh = 1000;
	PID_mv1_LimitLow = 0;
	PID_mv2_LimitHigh = 1000;
	PID_mv2_LimitLow =0 ;
	PID_alpha = 45;
	PID_beta = 100;
	PID_gamma = 30;

	PID_PDOF =0 ;
	PID_period = 10; // [10ms] 
	PID_disCompSwitch = 0;
}
void IEC::DL_SSM_CTRL_LIB::FB_PID_CONTROL::__Process(
    INT& PID_stage /*VAR_IN_OUT*/)
{

    uint32_t err_tmp = 0;
    uint32_t sv2dof_tmp = 0;   

    static CK_CONTROLLER controller_PID = ckCreateDtPidController();

    if (PID_stage == 1)
    {
      //------stage 1--------------//
        // PV=PV
        //set_controller_param(controller_PID, step_time);
        (ckControllerSetEnumParam(controller_PID, DT_PID_PARAM_PID_MODE, PID_mode));
        (ckControllerSetEnumParam(controller_PID, DT_PID_PARAM_CONTROL_ACTION, PID_controlAction));

        (ckControllerSetBoolParam(controller_PID, DT_PID_PARAM_DIS_COMP_SWITCH, PID_disCompSwitch));

        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_SV, SV));
        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_DEADBAND, PID_deadBand)); 
        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_D_LIMIT_GATE, PID_D_limitGate)); 
        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_STABLE_RANGE, PID_stableRange)); 

        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV1_LIMIT_HIGH, PID_mv1_LimitHigh));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV1_LIMIT_LOW, PID_mv1_LimitLow));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV2_LIMIT_HIGH, PID_mv2_LimitHigh));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV2_LIMIT_LOW, PID_mv2_LimitLow));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_PDOF, PID_PDOF));

        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_ALPHA, PID_alpha);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_BETA, PID_beta);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_GAMMA, PID_gamma);

        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_IPRELOAD, PID_I_preload);

        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_PB, PID_PB);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_TI, PID_TI);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_TD, PID_TD);

        int32_t period = (int32_t)(PID_period );  // unit: 10ms // 
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_PERIOD, period));

        ckControllerSetIntParam(controller_PID, DT_PID_PARAM_PV, PV);


        PID_stage = 2;
    }
    else if (PID_stage == 2)
    {
      //------stage 2--------------//

        ckControllerReset(controller_PID);

        ckControllerGetUIntParam(controller_PID, DT_PID_PARAM_ERR, &err_tmp);
        ckControllerGetUIntParam(controller_PID, DT_PID_PARAM_SV_2DOF, &sv2dof_tmp);
        
        PID_err = err_tmp;
	    PID_sv2DOF = sv2dof_tmp;
                  
        PID_stage = 3;
    }
    else if (PID_stage == 3)
    {
       // PV = PV
        //ckControllerSetIntParam(controller_PID, DT_PID_PARAM_SV, SV); //
       ckControllerSetIntParam(controller_PID, DT_PID_PARAM_PV, PV);
       
        //------stage 3--------------//
        ckControllerControl(controller_PID, &output_PID);
        MV1 = output_PID.mv[0];
        MV2 = output_PID.mv[1];
       
        //PID_stage = 4;
    }
    else if (PID_stage == 4)
    {
        //------stage 4--------------//
        //ckControllerFree(controller_PID);
        MV1 = 0;
        MV2 = 0;
        PID_stage = 0;
    }
  
}


//------------------------------------------------------------------------------------//

// FUNCTION_BLOCK FB_PID_FB_CONTROL
void IEC::DL_SSM_CTRL_LIB::FB_PID_FB_CONTROL::__Init()
{
	PID_controlAction = CK_CONTROL_ACTION_HEAT;
	PID_mode = 0;
	SV = 0;
	PV = 0;
    MV1 = 0;
    MV2 = 0;
	PID_deadBand = 0;
	PID_D_limitGate = 300;
	PID_stableRange = 0;
	PID_err = 0;
    PID_sv2DOF = 0;
    PID_I_preload = 0;
	PID_PB = 100;
	PID_TI = 100;
	PID_TD = 100;
	PID_cPB = 0;
	PID_cTI = 0;
	PID_cTD = 0;
	PID_mv1_LimitHigh = 1000;
	PID_mv1_LimitLow = 0;
	PID_mv2_LimitHigh = 1000;
	PID_mv2_LimitLow = 0;
	PID_alpha = 45;
	PID_beta = 100;
	PID_gamma = 30;

	PID_PDOF =0 ;
	PID_period = 2; // [10ms] 
	PID_disCompSwitch = 0;
}
void IEC::DL_SSM_CTRL_LIB::FB_PID_FB_CONTROL::__Process(
    INT& PID_stage /*VAR_IN_OUT*/)
{

    uint32_t err_tmp = 0;
    uint32_t sv2dof_tmp = 0;   
    static int32_t periodCount = 0;

    static CK_CONTROLLER controller_PID = ckCreateDtPidFbController();

    if (PID_stage == 1)
    {
      //------stage 1--------------//
        // PV=PV
        //set_controller_param(controller_PID, step_time);
        (ckControllerSetEnumParam(controller_PID, DT_PID_PARAM_PID_MODE, PID_mode));
        (ckControllerSetEnumParam(controller_PID, DT_PID_PARAM_CONTROL_ACTION, PID_controlAction));

        (ckControllerSetBoolParam(controller_PID, DT_PID_PARAM_DIS_COMP_SWITCH, PID_disCompSwitch));

        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_SV, SV));
        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_DEADBAND, PID_deadBand)); 
        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_D_LIMIT_GATE, PID_D_limitGate)); 
        (ckControllerSetIntParam(controller_PID, DT_PID_PARAM_STABLE_RANGE, PID_stableRange)); 

        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV1_LIMIT_HIGH, PID_mv1_LimitHigh));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV1_LIMIT_LOW, PID_mv1_LimitLow));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV2_LIMIT_HIGH, PID_mv2_LimitHigh));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_MV2_LIMIT_LOW, PID_mv2_LimitLow));
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_PDOF, PID_PDOF));

        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_ALPHA, PID_alpha);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_BETA, PID_beta);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_GAMMA, PID_gamma);

        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_IPRELOAD, PID_I_preload);

        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_PB, PID_PB);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_TI, PID_TI);
        ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_TD, PID_TD);

        int32_t period = (int32_t)(PID_period );  // unit: 10ms // 
        (ckControllerSetUIntParam(controller_PID, DT_PID_PARAM_PERIOD, period));
        periodCount = period;

        ckControllerSetIntParam(controller_PID, DT_PID_PARAM_PV, PV);


        PID_stage = 2;
    }
    else if (PID_stage == 2)
    {
      //------stage 2--------------//

        ckControllerReset(controller_PID);

        ckControllerGetUIntParam(controller_PID, DT_PID_PARAM_ERR, &err_tmp);
        ckControllerGetUIntParam(controller_PID, DT_PID_PARAM_SV_2DOF, &sv2dof_tmp);
        
        PID_err = err_tmp;
	    PID_sv2DOF = sv2dof_tmp;
                  
        PID_stage = 3;
    }
    else if (PID_stage == 3)
    {

        if (periodCount > 1)
        {
            periodCount--;            
        }
        else
        {
            // PV = PV
            //ckControllerSetIntParam(controller_PID, DT_PID_PARAM_SV, SV); //
            ckControllerSetIntParam(controller_PID, DT_PID_PARAM_PV, PV);
        
            //------stage 3--------------//
            ckControllerControl(controller_PID, &output_PID); // time base = 10ms
           
            MV1 = output_PID.mv[0];
            MV2 = output_PID.mv[1];
            
            periodCount = PID_period;
        }
        //PID_stage = 4;
    }
    else if (PID_stage == 4)
    {
        //------stage 4--------------//
        //ckControllerFree(controller_PID);
        MV1 = 0;
        MV2 = 0;
        PID_stage = 0;
    }
  
}







//----------------------------------------------------------------------------//
// FUNCTION_BLOCK FB_AT
void IEC::DL_SSM_CTRL_LIB::FB_AT::__Init()
{
	AT_controlAction = CK_CONTROL_ACTION_HEAT;
    SV = 0;
    PV = 0;
	AT_mv1_limitHigh = 1000 ;
	AT_mv2_limitHigh = 1000;
	AT_disCompSwitch = 0;
    AT_sv_ratio = 100 ;
    AT_pb_ratio = 100 ;
    AT_ti_ratio = 100;
    
    AT_pid_alpha = 100;
    AT_pid_beta = 100;
    AT_pid_gamma = 100;
    AT_pid_ipreload = 100;
    AT_timeBase = 1;

    MV1 = 0;
    MV2 = 0;
}


void IEC::DL_SSM_CTRL_LIB::FB_AT::__Process(INT& AT_stage /*VAR_IN_OUT*/ )
{
    uint32_t pbTemp=0, tiTemp=0, tdTemp=0;
    uint32_t alphaTemp=0, betaTemp=0, gammaTemp=0, ipreloadTemp=0;
       
    static CK_CONTROLLER controller_PID = ckCreateDtPidController();
    static CK_AUTO_TUNE autotune = ckCreateDtAutoTune();
    static uint16_t at_count = 0;

    if (AT_stage == 1)
    {   //------stage 1--------------//
        /*(set_auto_tune_param(autotune));
        //------
        (ckAutoTuneSetIntParam(autotune, DT_AUTO_TUNE_PARAM_SV, 1000));
        (ckAutoTuneSetEnumParam(autotune,DT_AUTO_TUNE_PARAM_CONTROL_ACTION,CK_CONTROL_ACTION_HEAT));
        (ckAutoTuneSetUIntParam(autotune,DT_AUTO_TUNE_PARAM_MV1_LIMIT_HIGH,1000));
        (ckAutoTuneSetUIntParam(autotune,DT_AUTO_TUNE_PARAM_MV2_LIMIT_HIGH,1000));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_SV_RATIO,100));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_PB_RATIO,100));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_TI_RATIO,100));
    */
        (ckAutoTuneSetIntParam(autotune, DT_AUTO_TUNE_PARAM_SV, SV));
        (ckAutoTuneSetEnumParam(autotune, DT_AUTO_TUNE_PARAM_CONTROL_ACTION, AT_controlAction));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_MV1_LIMIT_HIGH, AT_mv1_limitHigh));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_MV2_LIMIT_HIGH, AT_mv2_limitHigh));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_SV_RATIO, AT_sv_ratio));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_PB_RATIO, AT_pb_ratio));
        (ckAutoTuneSetUIntParam(autotune, DT_AUTO_TUNE_PARAM_TI_RATIO, AT_ti_ratio));
        (ckAutoTuneReset(autotune));
        at_count = 0;
        AT_stage = 2 ;

    } else if (AT_stage == 2)
    {
        //------stage 2--------------// take pv data
        // PV = PV

        ckAutoTuneCollectAndSignal(autotune, PV, &output_AT); // time base = 10ms
        //output value
        MV1 = output_AT.mv[0];
        MV2 = output_AT.mv[1];
        
        at_count = output_AT.mv_count;           

        if (at_count == 0 )
        {
            AT_stage = 3 ;
        }
    } else if (AT_stage == 3)
    {
      //------stage 3--------------//
      
        ckAutoTuneApplyParams(autotune, controller_PID); // 10[ms]

        get_pid_param(controller_PID, &pbTemp, &tiTemp, &tdTemp,
                      &alphaTemp, &betaTemp, &gammaTemp, &ipreloadTemp );

        //AT_pid_pb = (int)pbTemp *10 ;  // ch mark @20260806
        //AT_pid_ti = (int)tiTemp *100;
        //AT_pid_td = (int)tdTemp *100;

        AT_pid_pb = (int)pbTemp ; 
        AT_pid_ti = (int)tiTemp ;
        AT_pid_td = (int)tdTemp ;

        AT_pid_alpha = (int)alphaTemp;
        AT_pid_beta = (int)betaTemp;
        AT_pid_gamma = (int)gammaTemp;
        AT_pid_ipreload = (int)ipreloadTemp;

        AT_stage = 4;
    } else if (AT_stage == 4)
    {
        //ckAutoTuneFree(autotune); // ?

        MV1 = 0;
        MV2 = 0;
        AT_stage = 0;
    }


}


//----------------------------------------------------------------------------// 
// FUNCTION_BLOCK FB_ON_OFF_CONTROL
void IEC::DL_SSM_CTRL_LIB::FB_ON_OFF_CONTROL::__Init()
{
	onOff_controlAction = CK_CONTROL_ACTION_HEAT; 
    SV = 300 ;
	PV = 300;
	onOff_deadBand = 100;
	onOff_outTs1 = 40;
	onOff_outTs2 = 40;
	MV1 = 0;
	MV2 = 0;



   /* parameter.control_action = onOff_controlAction;
    parameter.deadband = onOff_deadBand;
    parameter.out_ts[0] = onOff_outTs1;
    parameter.out_ts[1] = onOff_outTs2;
    parameter.base.pv = PV;
    parameter.base.sv = SV;
*/
    
    
}
void IEC::DL_SSM_CTRL_LIB::FB_ON_OFF_CONTROL::__Process(
    INT& onOff_stage /*VAR_IN_OUT*/)
{
  /*
    parameter.control_action = onOff_controlAction;
    parameter.deadband = onOff_deadBand;
    parameter.out_ts[0] = onOff_outTs1;
    parameter.out_ts[1] = onOff_outTs2;
    parameter.base.pv = PV;
    parameter.base.sv = SV;

    CK_ERROR error = ckControllerControl((CK_CONTROLLER)controller,
                                           &parameter.base, &output);

  */

    static CK_CONTROLLER controller_onOff =  ckCreateDtOnOffController();
   // static CK_OUTPUT output = {0};

    if (onOff_stage == 1)
    {
        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_CONTROL_ACTION, onOff_controlAction);

        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_PV, PV);
        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_SV, SV);

        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_OUT1_TS, onOff_outTs1);
        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_OUT2_TS, onOff_outTs2);
        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_DEADBAND, onOff_deadBand);

        onOff_stage = 2;
    }
    else if (onOff_stage == 2)
    {
        onOff_stage = 3;
    } else if (onOff_stage == 3)
    {

        //------stage 3--------------//
        ckControllerControl(controller_onOff, &output_onOff);
        MV1 = output_onOff.mv[0];
        MV2 = output_onOff.mv[1];
        // PV = PV
        ckControllerSetIntParam(controller_onOff, DT_ON_OFF_PARAM_SV, SV);
        ckControllerSetIntParam(controller_onOff, DT_PID_PARAM_PV, PV);
        //onOff_stage = 4;
    } else if (onOff_stage == 4)
    {
        MV1 = 0;
        MV2 = 0;    
        onOff_stage = 0;
    }   
}

