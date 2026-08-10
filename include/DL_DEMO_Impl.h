#pragma once
#include "corner_abi/iec_type.h"
#include "cklib/inc/cklib.h"

namespace IEC
{
	namespace DL_SSM_CTRL_LIB
	{


	// FUNCTION_BLOCK FB_PID_FB_CONTROL
		class FB_PID_FB_CONTROL : public FUNCTION_BLOCK
		{
		public:
			FB_PID_FB_CONTROL() {};
			INT PID_controlAction; /*VAR_INPUT*/
			INT PID_mode; /*VAR_INPUT*/
			INT SV; /*VAR_INPUT*/
			INT PV; /*VAR_INPUT*/
			INT MV1; /*VAR_OUTPUT*/
			INT MV2; /*VAR_OUTPUT*/
			INT PID_deadBand; /*VAR_INPUT*/
			INT PID_D_limitGate; /*VAR_INPUT*/
			INT PID_stableRange; /*VAR_INPUT*/
			INT PID_err; /*VAR_OUTPUT*/
			INT PID_sv2DOF; /*VAR_OUTPUT*/
			INT PID_PB; /*VAR_INPUT*/
			INT PID_TI; /*VAR_INPUT*/
			INT PID_TD; /*VAR_INPUT*/
			INT PID_cPB; /*VAR_INPUT*/
			INT PID_cTI; /*VAR_INPUT*/
			INT PID_cTD; /*VAR_INPUT*/
			INT PID_mv1_LimitHigh; /*VAR_INPUT*/
			INT PID_mv1_LimitLow; /*VAR_INPUT*/
			INT PID_mv2_LimitHigh; /*VAR_INPUT*/
			INT PID_mv2_LimitLow; /*VAR_INPUT*/
			INT PID_alpha; /*VAR_INPUT*/
			INT PID_beta; /*VAR_INPUT*/
			INT PID_gamma; /*VAR_INPUT*/
			INT PID_I_preload; /*VAR_OUTPUT*/
			INT PID_PDOF; /*VAR_INPUT*/
			INT PID_period; /*VAR_INPUT*/
			INT PID_disCompSwitch; /*VAR_INPUT*/
            
			// BASIC METHOD
			void __Init();
            virtual void __Process(INT& PID_stage /*VAR_IN_OUT*/);

            CK_OUTPUT output_PID  = {0};
        };
                
        // FUNCTION_BLOCK FB_PID_CONTROL
		class FB_PID_CONTROL : public FUNCTION_BLOCK
		{
		public:
			FB_PID_CONTROL() {};
			INT PID_controlAction; /*VAR_INPUT*/
			INT PID_mode; /*VAR_INPUT*/
			INT SV; /*VAR_INPUT*/
			INT PV; /*VAR_INPUT*/
			INT MV1; /*VAR_OUTPUT*/
			INT MV2; /*VAR_OUTPUT*/
			INT PID_deadBand; /*VAR_INPUT*/
			INT PID_D_limitGate; /*VAR_INPUT*/
			INT PID_stableRange; /*VAR_INPUT*/
			INT PID_err; /*VAR_OUTPUT*/
			INT PID_sv2DOF; /*VAR_OUTPUT*/
			INT PID_PB; /*VAR_INPUT*/
			INT PID_TI; /*VAR_INPUT*/
			INT PID_TD; /*VAR_INPUT*/
			INT PID_cPB; /*VAR_INPUT*/
			INT PID_cTI; /*VAR_INPUT*/
			INT PID_cTD; /*VAR_INPUT*/
			INT PID_mv1_LimitHigh; /*VAR_INPUT*/
			INT PID_mv1_LimitLow; /*VAR_INPUT*/
			INT PID_mv2_LimitHigh; /*VAR_INPUT*/
			INT PID_mv2_LimitLow; /*VAR_INPUT*/
			INT PID_alpha; /*VAR_INPUT*/
			INT PID_beta; /*VAR_INPUT*/
			INT PID_gamma; /*VAR_INPUT*/
			INT PID_I_preload; /*VAR_OUTPUT*/
			INT PID_PDOF; /*VAR_INPUT*/
			INT PID_period; /*VAR_INPUT*/
			INT PID_disCompSwitch; /*VAR_INPUT*/
            
			// BASIC METHOD
			void __Init();
            virtual void __Process(INT& PID_stage /*VAR_IN_OUT*/);

            CK_OUTPUT output_PID  = {0};
        };

        // FUNCTION_BLOCK FB_AT
		class FB_AT : public FUNCTION_BLOCK
		{
		public:
			FB_AT() {} ;
			INT AT_controlAction; /*VAR_INPUT*/
			INT SV;               /*VAR_INPUT*/
			INT PV; /*VAR_INPUT*/

			INT AT_sv_ratio; /*VAR_INPUT*/
			INT AT_pb_ratio; /*VAR_INPUT*/
            INT AT_ti_ratio; /*VAR_INPUT*/
          	INT AT_mv1_limitHigh; /*VAR_INPUT*/
            INT AT_mv2_limitHigh; /*VAR_INPUT*/
            INT AT_disCompSwitch; /*VAR_INPUT*/
			INT AT_timeBase;/*VAR_INPUT*/
			INT AT_pid_pb; /*VAR_OUTPUT*/				
			INT AT_pid_ti; /*VAR_OUTPUT*/
			INT AT_pid_td; /*VAR_OUTPUT*/
			INT AT_pid_alpha; /*VAR_OUTPUT*/
			INT AT_pid_beta; /*VAR_OUTPUT*/
			INT AT_pid_gamma; /*VAR_OUTPUT*/
            INT AT_pid_ipreload; /*VAR_OUTPUT*/
            INT MV1; /*VAR_OUTPUT*/
			INT MV2; /*VAR_OUTPUT*/

			// BASIC METHOD
			void
			__Init();
			virtual void __Process(
                INT& AT_stage /*VAR_IN_OUT*/);

            CK_OUTPUT output_AT  = {0};    
        };
                
                
		// FUNCTION_BLOCK FB_ON_OFF_CONTROL
		class FB_ON_OFF_CONTROL : public FUNCTION_BLOCK
		{
		public:
			FB_ON_OFF_CONTROL() {};

            INT onOff_controlAction; /*VAR_INPUT*/
			INT SV; /*VAR_INPUT*/
			INT PV; /*VAR_INPUT*/
			INT onOff_deadBand; /*VAR_INPUT*/
			INT onOff_outTs1; /*VAR_INPUT*/
			INT onOff_outTs2; /*VAR_INPUT*/
			INT MV1; /*VAR_OUTPUT*/
			INT MV2; /*VAR_OUTPUT*/

          
			// BASIC METHOD
			void __Init();
            virtual void __Process(INT& onOff_stage /*VAR_IN_OUT*/);
           //DT_ON_OFF_CONTROLLER controller = ckCreateDtOnOffController();
  			//DT_ON_OFF_PARAM parameter = {0};
            CK_OUTPUT output_onOff  = {0};
		};	
 
	}
}
