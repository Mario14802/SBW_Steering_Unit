#include "Application.h"
#include <math.h>

// PI controller handle
PI_Handle_t PI_Handle;
float SP;
float PV;
float PWM_Out;
// value from interpolation
float Encoder_Ang;
float Linear_diplacement;

MB_Status_t Modbus_CallBack(void *context) {
    MB_Slave_t *mb = (MB_Slave_t *)context;
    // ... service your request, fill mb->Response[], set ResponseLength ...
    return MB_STAT_OK;
}
//----------------------------------------------------------------------------//
//                             CAN-related definitions                        //
//----------------------------------------------------------------------------//

static void Config_CAN_Filters(void) {
	const CAN_FilterTypeDef filter = { .FilterActivation = CAN_FILTER_ENABLE,
			.FilterBank = 10, .FilterFIFOAssignment = CAN_RX_FIFO0,
			.FilterIdHigh = 0x0000, .FilterIdLow = 0x0000, .FilterMaskIdHigh =
					0x0000, .FilterMaskIdLow = 0x0000, .FilterMode =
					CAN_FILTERMODE_IDMASK, .FilterScale =
			CAN_FILTERSCALE_32BIT, };

	HAL_CAN_ConfigFilter(&hcan1, &filter);
}

//----------------------------------------------------------------------------//
//                           Global Variable Declarations                      //
//----------------------------------------------------------------------------//
CAN_TxHeaderTypeDef myTX_header;
CAN_RxHeaderTypeDef myRX_header;
uint32_t myTxmailbox;
uint8_t myTxbuffer[8];
uint8_t myRxbuffer[8];
//received from feedback
uint8_t t;
int16_t Motor_current = 0;
int16_t steering_wheel_angle = 0;
int16_t Steering_wheel_speed = 0;
int16_t PWM_output = 0;
//sent to feedback
int16_t motor_current = 0;
int16_t rack_position = 0;
int16_t rack_force = 0;
//int16_t PWM_Stee;

//----------------------------------------------------------------------------//
//                             Application Initialization                     //
//----------------------------------------------------------------------------//
void Application_Init(void) {
	//-------------- Modbus initialization --------------//
	MB_Init_UART1(&huart1, 0x1);
	MB_Init_USB_MODBUS(&hUsbDeviceFS,0x2);

	MB.hw_interface.MB_Request_Recieved=&Modbus_CallBack;
		USB_MB.hw_interface.MB_Request_Recieved=&Modbus_CallBack;

		free(USB_MB.HoldingRegs);
		free(USB_MB.InputRegs);
		free(USB_MB.InputBits);
		free(USB_MB.CoilBits);

		USB_MB.HoldingRegs=MB.HoldingRegs;
		USB_MB.InputRegs=MB.InputRegs;
		USB_MB.InputBits=MB.InputBits;
		USB_MB.CoilBits=MB.CoilBits;

	//-------------- CAN initialization --------------//

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	Config_CAN_Filters();

	myTX_header.DLC = 8;
	//myTX_header.ExtId = 0;
	myTX_header.IDE = CAN_ID_STD;
	myTX_header.RTR = CAN_RTR_DATA;
	myTX_header.StdId = Sender_ID; //id of the sender
	myTX_header.TransmitGlobalTime = DISABLE;
	memset(myTxbuffer, 0, Txsize);

	//-------------- Motor timer (TIM8) initialization --------------//
	HAL_TIM_Base_Start(&htim8);
	Motor_Init(&htim8, TIM_CHANNEL_1);
	Motor_Init(&htim8, TIM_CHANNEL_2);

	//-------------- ADC trigger via TIM5 / DMA --------------//
	//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); //that triggers the ADC conversion
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) Iregs->ADC_Raw_Values, 9);

	//LOAD default
	if (!Load_NV_Data()) {
		Load_Default_NV_Data();
	}

	//-------------- PI controller initialization --------------//
	PI_Init(&PI_Handle, DefaultParams.Control_Max_Out,
			DefaultParams.I_Control_Kp, DefaultParams.I_Control_Ki,
			DefaultParams.I_Control_Kc, DefaultParams.Controller_Sampling_Time);
	HAL_TIM_Base_Start_IT(&htim6); //to call the pi evaluat fun every 5 ms

	//--------------Interpolation initialization --------------//
	/*	//ENCODER VALUES
	 M->output_max = 75.0f;
	 M->output_min = -75.0f;
	 //LINEAR VALUES
	 M->input_max = 180.0f;
	 M->input_min = -180.0f;*/
}

//----------------------------------------------------------------------------//
//                             Main Application Loop                          //
//----------------------------------------------------------------------------//
inline void Application_Run(void) {
	//uint32_t Ticks = HAL_GetTick();
	uint32_t PID_Ticks = HAL_GetTick();
	//
	//HAL_StatusTypeDef S;

	while (1) {
		// Modbus routine
		MB_Slave_Routine(&MB, HAL_GetTick());

		MB_Slave_Routine(&USB_MB, HAL_GetTick());

		Encoder_Ang = map_linear(M, (float) steering_wheel_angle);

		//ADC VALUES
		Compute_Analog_Measurements();

		//change form 0-200 to -75,75mm
		Linear_diplacement = get_linear_position(Iregs->Linear_position);

		//Sent Data
		Motor_current = (int16_t) Iregs->I_OUT;
		rack_position = (int16_t) Linear_diplacement;
		//rack_Force

		// PI control update every 5 ms

		uint32_t HAL_Tick = HAL_GetTick();
		if (HAL_Tick >= PID_Ticks) {
			if (GetCoil(MB_Coil_Enable_PI_Controller)) {

				SP = (float) Encoder_Ang;
				PV = Linear_diplacement;
				Iregs->Motor_LP_SP = SP;

				PWM_Out = PI_Eval(&PI_Handle, SP, // Sp is the desired value from the interoplation
						PV      				 //  actual Linear Length
						);

				Iregs->Motor_PWM_Out = PWM_Out;
				Iregs->Motor_D_Error = PI_Handle.Error;

				if (PWM_Out > 0) {
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
							(uint16_t ) PWM_Out);
				} else {

					PWM_Out = fabsf(PWM_Out);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
							(uint16_t )PWM_Out);
				}

			} else {
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			}
			PID_Ticks = HAL_GetTick() + 5;  // qadded 5ms

		}

		// Prepare and send CAN message
		PrepareCANMessage(myTxbuffer, motor_current, rack_position, rack_force);

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3) {
			if (HAL_CAN_AddTxMessage(&hcan1, &myTX_header, myTxbuffer,
					&myTxmailbox) != HAL_OK) {
				Error_Handler();  // Handle error if sending fails
			}
		}

		if (GetCoil(MB_Coil_Update_Params)) {
			PI_Handle.KP = Hregs->sParams.I_Control_Kp;
			PI_Handle.KI = Hregs->sParams.I_Control_Ki;
			//SetCoil(MB_Coil_Update_Params, 0);
		}

		if (GetCoil(MB_Coil_Update_MAXOUT)) {
			PI_Handle.OutMax = Hregs->sParams.Control_Max_Out;
			SetCoil(MB_Coil_Update_MAXOUT, 0);
		}

		if (GetCoil(MB_Coil_Update_KC)) {
			PI_Handle.OutMax = Hregs->sParams.I_Control_Kc;
			SetCoil(MB_Coil_Update_KC, 0);
		}

		if (GetCoil(MB_Coil_Load_Defaults)) {
			Load_Default_NV_Data();
			SetCoil(MB_Coil_Load_Defaults, 0);
		}
	}
}

//----------------------------------------------------------------------------//
//                          ADC Measurement Computation                       //
//----------------------------------------------------------------------------//
void Compute_Analog_Measurements(void) {
	//calculate the Vbus voltage
	Iregs->Vbus = ((float) Iregs->ADC_Raw_Values[1]
			* (DefaultParams.Vmotor_Sense_Gain))
			- (DefaultParams.Vmotor_Sense_Offset);
	Iregs->I_OUT = ((float) Iregs->ADC_Raw_Values[0]
			* (DefaultParams.I_Sense_Gain)
			- (DefaultParams.I_Sense_Offset + DefaultParams.Amplifier_offset))
			* 10.0f;

	Iregs->Linear_position = ((float) Iregs->ADC_Raw_Values[7]); //*(200.0f/4095.0f);//linear  position sensor

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// Currently empty—add logic here if needed.
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef myRX_header;
	uint8_t myRxbuffer[8];
	// Decode based on your DBC layout
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &myRX_header, myRxbuffer)
			== HAL_OK) {
		// Only process if message ID is 0x101 (Standard ID)
		if (myRX_header.StdId == 0x101/* && myRX_header.IDE == CAN_ID_STD*/) {

			Motor_current = decode_data(myRxbuffer, 0, 13, 1);
			steering_wheel_angle = decode_data(myRxbuffer, 13, 12, 1);
			Steering_wheel_speed = decode_data(myRxbuffer, 25, 14, 1);
			PWM_output = decode_data(myRxbuffer, 39, 15, 1); // Fixed: now using next 16 bits after speed

			// TODO: Use or store the values as needed
		}
	} else {
		// Optional: Add error handling if needed
	}
}
