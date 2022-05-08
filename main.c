/******************************************************************************
* File Name:   main.c
*
* Description: + This is the source code for the PDM PCM Example
*              for ModusToolbox.
*              + The Interrupt handler take the Signal from the PDM-Mic
*              and push to the Small-Window. When the Small-Window is
*              full. Set the Transformation-flag.
*              + In main, when Transformation-flag is set, Element from
*              Small-Window is pushed into Big-Window (FIFO). Then it
*              starts the Transformation-Process
*              + The Transformation Process is written in C, based on
*              https://github.com/ARM-software/ML-KWS-for-MCU/tree/master/Deployment/Source/MFCC
*              which is written in C++.
*              + Using DTW and KNN to find the result
*              + To change the Configuration: change the #Define variable
* TO DO    	 : + EEPROM to save the Exemplar when the Board is off.
* 			   + #define 	ARRAY_EEPROM_SIZE	3100 / 31 * Struct MFCC_ID(100bytes)
* 			   				ARRAY_EEPROM_START	0 ( 0 - 3099)
* 			   				COUNT_EEPROM_SIZE 	1 (1 * uint8_t)
* 			   				COUNT_EEPROM_START 	3100
*
*
*			For Em_EEPROM: 	Em_EEPROM_PHYSICAL_SIZE		(2048u)
* 			   				EEPROM_SIZE					(256u)
* 			   				BLOCKING_WRITE				(1u)
* 			   				REDUNDANT_COPY				(1u)
* 			   				WEAR_LEVELLING_FACTOR		(2u)
*
* 			HandelError:   	STATUS_SUCCESS				(0u)
* 							void HandleError(uint32_t status, char *message) -> to print message
*
* 			EEPROM configuration and context structure:
* 							cy_stc_eeprom_config_t Em_EEPROM_config
* 							cy_stc_eeprom_context_t Em_EEPROM_context
*
* 			Create EEPROM:	const uint8_t EepromStorage[Em_EEPROM_PHYSICAL_SIZE] = {0u}
*
* 			Array to Read and Write to EEPROM:
* 				can use: 	struct MFCC_ID exemplar_list[EXEMPLAR_NUMBER] ~31*100 bytes ???
* 				or:			uint8_t eepromReadArray[ARRAY_EEPROM_SIZE] ??? must be same start address with exemplar_list ( Bytes copy)
*				using Address / Pointer
*
*			enum value:		cy_en_em_eeprom_status_t eepromReturnValue
*
*			Initialize the flash start address in EEPROM configuration structure:
*							Em_EEPROM_config.userFlashStartAddr = (uint32_t)EepromStorage;
*							eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);
*							HandleError(eepromReturnValue, "Emulated EEPROM Initialization Error \r\n");
*
*			Read Array(31*100 bytes) + Count(1 byte) From EEPROM when the Board is turn on (where to put this code):
*							eepromReturnValue = Cy_Em_EEPROM_Read(ARRAY_EEPROM_START, eepromReadArray,
*																  ARRAY_EEPROM_SIZE, &Em_EEPROM_context);
*							eepromReturnValue = Cy_Em_EEPROM_Read(COUNT_EEPROM_START, E_count,
*																  COUNT_EEPROM_SIZE, &Em_EEPROM_context);
*
*			Write to EEPROM when new Exemplar is added:
*							RESET_COUNT_LOCATION = E_count * 100;
*							eepromReturnValue = Cy_Em_EEPROM_Write(RESET_COUNT_LOCATION,
*																   &eepromReadArray[RESET_COUNT_LOCATION],
*																   RESET_COUNT_SIZE,
*																   &Em_EEPROM_context);
*
*
*
* Related Document: See Readme.md
*
*
*******************************************************************************
* (c) (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#define ARM_MATH_CM4 // define which uC we used
//#define __arm__
//#define ARMCM4_FP

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cybsp_types.h"
#include "cy_serial_flash_qspi.h"
#include "cycfg_qspi_memslot.h"
#include "mem_config_sfdp.h"

#include "stdio.h"
#include "stdlib.h"
#include "arm_math.h" // library for DSP-RFFT
#include "string.h"
#include "float.h"
#include "dtw.h"
/*******************************************************************************
* Macros
********************************************************************************/
/* Trigger level configured in the PDM/PCM */
#define PDM_PCM_FIFO_TRG_LVL        1u
/* Sample rate == change by HFCLK1 (Device Configuration)*/
#define SAMPLE_RATE 				4000u
/* Noise threshold hysteresis */
#define THRESHOLD_HYSTERESIS        400u //<- Change threshold base on Sound-volume
/* Number of Sample of Big Window */
#define MAX_BIG_WINDOW				1200
/* Number of Sample of Small Window */
#define MAX_SMALL_WINDOW 			200u//100u - set transformation flag
/* Different from Number of Sample */
#define DIF_SAMPLE					MAX_BIG_WINDOW - MAX_SMALL_WINDOW
/* Number Sample of FFT */
#define NFFT 						256u//128u //When change NFFT -> change function arm_rfft_NFFT_fast_init_f32 / arm_status
/* Frame-step from each PCM-DATA input to FFTR*/
#define FRAME_STEP 					150u//150u//100u//50


/* Number of MFCC */
#define N_MFCC 						12
/* MFCC column */
#define MFCC_COL MAX_BIG_WINDOW/FRAME_STEP // 16

#define NUM_FBANK_BINS 16//30//18
#define MEL_LOW_FREQ 20
#define MEL_HIGH_FREQ SAMPLE_RATE/2 // Because Nyquist frequency
#define M_2PI 6.283185307179586476925286766559005

#define EXEMPLAR_NUMBER 16

#define NUMBER_OF_LAST_SAMPLE 7 //Create an Array to check last n Samples

#define K_sample 3 		// for K-nearest neighbour -> each label should have K_sample samples
#define MAX_MFCC_ID 8 	// MFCC_ID muss < MAX_MFCC_ID

#define mfcc_dec_bits  7                            // using in Constructor in C++

//Macro for Flash
#define MEM_SLOT_NUM            (0u)      /* Slot number of the memory to use */
#define QSPI_BUS_FREQUENCY_HZ   (50000000lu)
#define ARRAY_ADDRESS 	0x02u
#define ARRAY_SIZE		STRUCT_SIZE * EXEMPLAR_NUMBER
#define COUNT_ADDRESS	0x00u

//TON Block for cool down time from LED
#define TON_TIME 	SAMPLE_RATE/MAX_SMALL_WINDOW
/*******************************************************************************
* Function Prototypes
********************************************************************************/
void button_isr_handler(void *callback_arg, cyhal_gpio_event_t event);
void pdm_pcm_isr_handler(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Interrupt flags */
volatile bool button_flag = false;
volatile bool pdm_pcm_flag = false;
volatile bool transformation_flag = false;
volatile bool record_flag = false;
volatile bool finish_record = false;
uint16_t count = 0; //check the speed

/* Volume variables */
uint32_t volume = 0;
uint32_t num_samples = 0;

const cy_stc_sysint_t pdm_pcm_isr_cfg = {
#if CY_IP_MXAUDIOSS_INSTANCES == 1
    .intrSrc = (IRQn_Type) audioss_interrupt_pdm_IRQn,
#else
    .intrSrc = (IRQn_Type) audioss_0_interrupt_pdm_IRQn,
#endif
    .intrPriority = CYHAL_ISR_PRIORITY_DEFAULT
};

/* Initialization */
int16_t smallWindow[MAX_SMALL_WINDOW] = {0};
int16_t bigWindow[MAX_BIG_WINDOW] = {0};

uint8_t inputSmallWindow = 0;

/* Create Struct-Type for MFCC-data with ID (sound label) and MFCC-data */
struct MFCC_ID{							//now: 100 bytes
	uint16_t soundID;					//2 bytes
	int8_t mfcc[N_MFCC][MFCC_COL];		//MFCC_COL x N_MFCC bytes
	uint16_t dtw_distance;				//2 bytes
};

#define STRUCT_SIZE		sizeof(struct MFCC_ID)

uint8_t exemplar_index = 1;

//all the array here are static in the board
float32_t frame[NFFT];
float32_t buffer[NFFT];
float mel_energies[NUM_FBANK_BINS];

//create window function
float window_func[NFFT];

//create mel filterbank
int32_t fbank_filter_first[NUM_FBANK_BINS];
int32_t fbank_filter_last[NUM_FBANK_BINS];
float mel_fbank[NUM_FBANK_BINS][NFFT/2];
//create DCT matrix
float dct_matrix[NUM_FBANK_BINS*N_MFCC];
//initialize FFT
arm_rfft_fast_instance_f32 rfft;


//static inline float InverseMelScale(float);
static inline float MelScale(float);
void init_window_function(float *);
void create_mel_fbank(float (*)[NFFT/2], int32_t * , int32_t * );
void create_dct_matrix(float *);
void mfcc_compute(const int16_t *, q7_t*);
void array_mfcc_compute(arm_status , int16_t * , int16_t * , q7_t * , int8_t (*)[MFCC_COL]);

//for k-NN array
//finding most appear
int max_index_int(int *a, int n);
int numToRepeatMax(int* , int , int );
int maxAppear(int8_t* arr1 , int n, int k);

//output signal to control LED
float smallest_distance;
uint8_t output_signal = 0;

/*******************************************************************************
* Function Name: check_status
****************************************************************************//**
* Summary:
*  Prints the message, indicates the non-zero status by turning the LED on, and
*  asserts the non-zero status.
*
* Parameters:
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
*******************************************************************************/
void check_status(char *message, uint32_t status)
{
    if(0u != status)
    {
        printf("\r\n================================================================================\r\n");
        printf("\nFAIL: %s\r\n", message);
        printf("Error Code: 0x%08lX\r\n", status);
        printf("\r\n================================================================================\r\n");

        /* On failure, turn the LED ON */
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        while(true); /* Wait forever here when error occurs. */
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function for Cortex-M4 CPU does the following:
*  Initialization:
*  - Initializes all the hardware blocks
*  Do forever loop:
*  - Check if PDM/PCM flag is set. If yes, report the current volume
*  - Update the LED status based on the volume and the noise threshold
*  - Check if the User Button was pressed. If yes, reset the noise threshold
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize Flash Memory*/
    result = cy_serial_flash_qspi_init(smifMemConfigsSfdp[MEM_SLOT_NUM], CYBSP_QSPI_D0,
                  CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC,
                  CYBSP_QSPI_SCK, CYBSP_QSPI_SS, QSPI_BUS_FREQUENCY_HZ);
    check_status("Serial Flash initialization failed", result);

    /* Initialize the User LED */
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize the LEDs */
	cyhal_gpio_init((cyhal_gpio_t)CYBSP_LED1_RED, CYHAL_GPIO_DIR_OUTPUT,
					CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	cyhal_gpio_init((cyhal_gpio_t)CYBSP_LED2_GREEN, CYHAL_GPIO_DIR_OUTPUT,
						CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	cyhal_gpio_init((cyhal_gpio_t)CYBSP_LED3_YELLOW, CYHAL_GPIO_DIR_OUTPUT,
						CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	cyhal_gpio_init((cyhal_gpio_t)CYBSP_RGB1_GREEN, CYHAL_GPIO_DIR_OUTPUT,
						CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	cyhal_gpio_init((cyhal_gpio_t)CYBSP_RGB2_RED, CYHAL_GPIO_DIR_OUTPUT,
						CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	cyhal_gpio_init((cyhal_gpio_t)CYBSP_RGB3_BLUE, CYHAL_GPIO_DIR_OUTPUT,
						CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);


    /* Initialize the User Button */
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event((cyhal_gpio_t) CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback((cyhal_gpio_t) CYBSP_USER_BTN, button_isr_handler, NULL);

    /* Initialize the PDM/PCM interrupt (PDL) */
    Cy_SysInt_Init(&pdm_pcm_isr_cfg, pdm_pcm_isr_handler);
    NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);

    /* Initialize the PDM/PCM block (PDL) */
    Cy_PDM_PCM_Init(CYBSP_PDM_PCM_HW, &CYBSP_PDM_PCM_config);
    Cy_PDM_PCM_ClearFifo(CYBSP_PDM_PCM_HW);
    Cy_PDM_PCM_Enable(CYBSP_PDM_PCM_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    //arm_status status = arm_rfft_fast_init_f32(&rfft, NFFT); <- here return ERROR arm_status
    arm_status status = arm_rfft_256_fast_init_f32(&rfft);
    init_window_function(window_func);
    create_mel_fbank(mel_fbank, fbank_filter_first, fbank_filter_last);
    create_dct_matrix(dct_matrix);
    int16_t in[NFFT];
    q7_t data_out[N_MFCC];

    uint8_t record_count = 0;

    int16_t record_in[NFFT];
    q7_t record_data_out[N_MFCC];

    uint8_t  counter_check;

    struct MFCC_ID exemplar_list[EXEMPLAR_NUMBER];
    struct MFCC_ID record_MFCC;
    struct MFCC_ID real_time_MFCC;
/*    create an uint8_t pointer, which point at the start of MFCC_ID array and a pointer for counter*/
    uint8_t * array_pointer = &exemplar_list;
    uint8_t * counter_pointer = &exemplar_index;

    int16_t record[MAX_BIG_WINDOW] = {0};

//    result = cy_serial_flash_qspi_read(COUNT_ADDRESS, 1, &exemplar_index);
	result = cy_serial_flash_qspi_read(COUNT_ADDRESS, 1, counter_pointer);
    check_status("read counter 1 failed", result);
    if(exemplar_index == 0xFFu || exemplar_index == 0x00u){		// if exemplar_index = default value (new loading code) -> set exemplar_index = 0;
    	size_t sectorSize = cy_serial_flash_qspi_get_erase_size(ARRAY_ADDRESS);
    	result = cy_serial_flash_qspi_erase(COUNT_ADDRESS, sectorSize);
    	check_status("Erasing memory failed", result);

    	exemplar_index = 0;

		array_mfcc_compute(status, record, record_in, record_data_out, record_MFCC.mfcc);
		record_MFCC.soundID = 0;

		exemplar_list[exemplar_index++] = record_MFCC;

		result = cy_serial_flash_qspi_write(COUNT_ADDRESS, 1, &exemplar_index);
//		result = cy_serial_flash_qspi_write(COUNT_ADDRESS, 1, counter_pointer);
		check_status("write counter 1 failed", result);


//		cy_serial_flash_qspi_write(ARRAY_ADDRESS+exemplar_index*STRUCT_SIZE,STRUCT_SIZE,&exemplar_list[exemplar_index]); 	//write new struct to Flash
//      cy_serial_flash_qspi_write(ARRAY_ADDRESS+exemplar_index*STRUCT_SIZE,STRUCT_SIZE, array_pointer + exemplar_index*STRUCT_SIZE;	//write new struct
//		cy_serial_flash_qspi_write(ARRAY_ADDRESS, ARRAY_SIZE, (uint8_t *) exemplar_list);		//write a whole exemplar list to Flash
		result = cy_serial_flash_qspi_write(ARRAY_ADDRESS, ARRAY_SIZE, array_pointer);
		check_status("write array 1 failed", result);					//write array through uint8_t array_pointer
    }
//    cy_serial_flash_qspi_read(ARRAY_ADDRESS, ARRAY_SIZE, (uint8_t *) exemplar_list);			//copy the exemplar_list from Flash
	result = cy_serial_flash_qspi_read(ARRAY_ADDRESS, ARRAY_SIZE, array_pointer);						//read array through uint8_t array_pointer
    check_status("read array 1 failed", result);

    int8_t lastSampleArray[NUMBER_OF_LAST_SAMPLE];
    int8_t sampleArrayCounter = 0;

//    TON Block, Cool down for change LED Status
    int8_t tonTimer = 20;
    uint8_t lightmodus = 0;
    printf("****************** \
    Projekt \
    ****************** \r\n\n");

    for(;;)
    {
    	/* Check if transformation_flag is set */
        if (transformation_flag)
        {
        	/* Clear the Transformation_flag */
        	transformation_flag = false;
//        	If Button_flag = True, turn on the USER_LED, disable the transformation process
        	if(button_flag == true){
        		//cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_ON);
//        		If record_flag is not set, do nothing
        		if(record_flag == true){
        			//if
        			uint16_t index_record = record_count++ * MAX_SMALL_WINDOW;
        			for(uint16_t i = 0; i < MAX_SMALL_WINDOW; i++){
        				if((i + index_record) < MAX_BIG_WINDOW){
        					record[i + index_record] = smallWindow[i];
        				}
        				else{ // Break the loop when the record array is full
        					finish_record = true;
        					break;
        				}
        			}
//        			When finished the record-process, let the record through the transform
//					process set ID through scanf(), save it in the Example-Array.
//        			reset: button_flag, record_flag, finish_record.
//        			set record_count = 0
        			if(finish_record){
        				array_mfcc_compute(status, record, record_in, record_data_out, record_MFCC.mfcc);
        				printf("\n\r finish record");
        				for(int i = 0; i < MAX_BIG_WINDOW; i++){
							printf("\n\r        %d", record[i]);
						}
//                        printf("\n\r    sound-ID");
                        scanf("%hi", &record_MFCC.soundID);
                        //Here is the end of record-process, copy the record to the exemplar_list of MFCC
                        exemplar_list[exemplar_index++] = record_MFCC;
                        record_count = 0;
                        button_flag = false;
                        record_flag = false;
                        finish_record = false;
                        /*Update new Array and new Counter to Flash*/
//                		result = cy_serial_flash_qspi_write(COUNT_ADDRESS, 1, &exemplar_index);			//write new counter
                		result = cy_serial_flash_qspi_write(COUNT_ADDRESS, 1, counter_pointer);
                		check_status("write counter failed", result);


//                		cy_serial_flash_qspi_write(ARRAY_ADDRESS+exemplar_index*STRUCT_SIZE,STRUCT_SIZE,&exemplar_list[exemplar_index]); 	//write new struct to Flash
//                		cy_serial_flash_qspi_write(ARRAY_ADDRESS+exemplar_index*STRUCT_SIZE,STRUCT_SIZE, array_pointer + exemplar_index*STRUCT_SIZE;	//write new struct
//                		cy_serial_flash_qspi_write(ARRAY_ADDRESS, ARRAY_SIZE, (uint8_t *) exemplar_list);									//write a whole exemplar list to Flash
                		result = cy_serial_flash_qspi_write(ARRAY_ADDRESS, ARRAY_SIZE, array_pointer);												//write array through uint8_t array_pointer
                        check_status("write array failed", result);
        			}
        		}
        	}
        	else{
                //cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
    			/* Push the whole Small Window to Big Window and Transform */
    			for(uint16_t index_shift_big_window = 0; index_shift_big_window < DIF_SAMPLE; index_shift_big_window++){
    				bigWindow[index_shift_big_window] = bigWindow[index_shift_big_window + MAX_SMALL_WINDOW];
    			}
    			for(uint8_t index_add_small_to_big_window = 0; index_add_small_to_big_window < MAX_SMALL_WINDOW; index_add_small_to_big_window++){
    				bigWindow[index_add_small_to_big_window + DIF_SAMPLE] = smallWindow[index_add_small_to_big_window];
    			}
    			/* Transform...*/
    			array_mfcc_compute(status, bigWindow, in, data_out, real_time_MFCC.mfcc);
//    			finish MFCC-process, start to compare with the exemplar, with the help of DTW

//    			do the DTW function only when already have a record.
    			if(exemplar_index > 0){
    				for(uint8_t dtw_index = 0; dtw_index < exemplar_index; dtw_index++){
						uint32_t summe = 0;
						for(uint8_t mfcc_index = 0; mfcc_index < N_MFCC; mfcc_index++){
							summe += dtwOneDimSameSize(MFCC_COL, real_time_MFCC.mfcc[mfcc_index], exemplar_list[dtw_index].mfcc[mfcc_index]);
						}
						exemplar_list[dtw_index].dtw_distance = summe;
					}

    				int dtw_ID[K_sample] = {0};
    				int dtw_distance_kNN[K_sample];
    				for(uint8_t i = 0; i < K_sample; i++){
    					dtw_distance_kNN[i] = 10000;
    				}

					for(uint8_t finding_index = 0; finding_index < exemplar_index; finding_index++){
						int index_number = max_index_int(dtw_distance_kNN, K_sample);
						if(exemplar_list[finding_index].dtw_distance < dtw_distance_kNN[index_number]){
							dtw_distance_kNN[index_number] = exemplar_list[finding_index].dtw_distance;
							dtw_ID[index_number] = exemplar_list[finding_index].soundID;
						}
					}
//					find which elements appear the most in dtw_ID
					output_signal = numToRepeatMax(dtw_ID, K_sample, MAX_MFCC_ID);
					printf("\n\routput_signal: %d", output_signal);
					result = cy_serial_flash_qspi_read(COUNT_ADDRESS, 1, &counter_check);
					check_status("counter check failed", result);
					printf("	Flash Counter: %d", counter_check);
				}
//    			Recognition for Whistle
//    			0 is for Noise, 1 is for Whistle.
				lastSampleArray[sampleArrayCounter++] = output_signal;
				if(sampleArrayCounter >= NUMBER_OF_LAST_SAMPLE){
					sampleArrayCounter = 0;
				}
//				here is only for whistle / 1 sound recognition, when more sounds. using function max_index_int under
				int8_t RecognitionSum = 0;
				for(int8_t i = 0; i < NUMBER_OF_LAST_SAMPLE; i++){
					RecognitionSum += lastSampleArray[i];
				}

				if(RecognitionSum > 4 && tonTimer > TON_TIME){
					printf("\n\r                                   Whistle %d",lightmodus);
					tonTimer = 0;
//					Cy_GPIO_Inv(PROJEKT_LED_PORT, PROJEKT_LED_PIN);
					cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_USER_LED);
					lightmodus++;
				}

/*				Test more sample
				int sound_result = numToRepeatMax(lastSampleArray,NUMBER_OF_LAST_SAMPLE,4);
				int sound_appearance = maxAppear(lastSampleArray,NUMBER_OF_LAST_SAMPLE,4);
				if(sound_result == 1 && sound_appearance > 4){
					printf("\n\r                                   Whistle");
				}
				if(sound_result == 2 && sound_appearance > 4){
					printf("\n\r                                   Clap");
				}
				if(sound_result == 3 && sound_appearance > 4){
					printf("\n\r                                   Cough");
				}*/

    			/*						*/
					//Control LEDs here
/***********************************LED funktions**********************************************************************/

				/* Lightmodus 1 all LEDs are on*/
				if(lightmodus == 1)
				{
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED1_RED, CYBSP_LED_STATE_ON);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED2_GREEN, CYBSP_LED_STATE_ON);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED3_YELLOW, CYBSP_LED_STATE_ON);
				}

/***********************************RGB funktions**********************************************************************/

				/*Lightmodus 2 turns the red on*/
				if(lightmodus == 2)
				{	/*turns the green on*/
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB1_GREEN, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB2_RED, CYBSP_LED_STATE_ON);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB3_BLUE, CYBSP_LED_STATE_OFF);
				}

				/*Lightmodus 3 turns the green on*/
				if(lightmodus == 3)
				{
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB1_GREEN, CYBSP_LED_STATE_ON);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB2_RED, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB3_BLUE, CYBSP_LED_STATE_OFF);
				}

				/*Lightmodus 4 turns the blue on*/
				if(lightmodus == 4)
				{
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB1_GREEN, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB2_RED, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB3_BLUE, CYBSP_LED_STATE_ON);
				}
				/*Light modus 5 blinking LED*/
				if(lightmodus == 5)
				{
					cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_LED1_RED);
					cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_LED2_GREEN);
					cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_LED3_YELLOW);
				}

				/*Lightmodus is bigger than 6 puts it back at zero*/
				if(lightmodus >= 6)
				{
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED1_RED, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED2_GREEN, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_LED3_YELLOW, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB1_GREEN, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB2_RED, CYBSP_LED_STATE_OFF);
					cyhal_gpio_write((cyhal_gpio_t)CYBSP_RGB3_BLUE, CYBSP_LED_STATE_OFF);
					lightmodus = 0;
				}



    			/*						*/
//              turn off the state change function with TON-Block for ~1s
				if(tonTimer <= TON_TIME)
					tonTimer = tonTimer + 1;
    			/*...*/

        	}

        }
    }
}

/*******************************************************************************
* Function Name: button_isr_handler
********************************************************************************
* Summary:
* Button ISR handler. Set a flag to be processed in the PDM-PCM Interrupt.
* Record new exemplar.
*
* Parameters:
*  callback_arg: not used
*  event: event that occured
*
*******************************************************************************/
void button_isr_handler(void *callback_arg, cyhal_gpio_event_t event)
{
    (void) callback_arg;
    (void) event;

    button_flag = true;
}

/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
* PDM/PCM ISR handler. Push the new Volume-info in the smallArray
* If enough elements in Array, set Flag to process transformation in main loop
*
*******************************************************************************/
int pdm_counter = 0;
void pdm_pcm_isr_handler(void)
{
    /* Disable PDM/PCM ISR to avoid multiple calls to this ISR */
    NVIC_DisableIRQ(pdm_pcm_isr_cfg.intrSrc);

	volume = abs((int16_t) Cy_PDM_PCM_ReadFifo(CYBSP_PDM_PCM_HW));

//	check for sample rate = 4000, print frequency 1hz
	count += 1;
	if(count >= SAMPLE_RATE){
		printf("\n		Change %d\r", exemplar_index);
//		cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_USER_LED);
		count = 0;
	}

	/*    If the Button_flag is set and the volume greater than threshold, set Record_flag -> starting record, checking main loop */
	if(exemplar_index < K_sample  * 2 + 1){ // Take total K_sample * 2 (silence + normal noise)
		//Here to take normal noise
		if(/*volume >= THRESHOLD_HYSTERESIS &&*/ button_flag == true){
			record_flag = true;
		}
	}
	else{
		//Here to take the noise greater than threshold
		if(volume >= THRESHOLD_HYSTERESIS && button_flag == true){
			record_flag = true;
		}
	}
//    	printf("\n\r Volume = %d", volume);

	/* Push the new Volume to Small Window each cycle */
	if(inputSmallWindow < MAX_SMALL_WINDOW){
		smallWindow[inputSmallWindow] = volume ;
		inputSmallWindow++;
//		 set transformation_flag to process in main for(;;)
		if(inputSmallWindow >= MAX_SMALL_WINDOW){
			transformation_flag = true;
		}
	}
	else{
		/* Start new Small Window*/
		inputSmallWindow = 0;
		smallWindow[0] = volume;
		inputSmallWindow += 1;
	}
	volume = 0;

	/* Clear the PDM/PCM interrupt */
	Cy_PDM_PCM_ClearInterrupt(CYBSP_PDM_PCM_HW, CY_PDM_PCM_INTR_RX_TRIGGER);
    /* Re-enable PDM/PCM ISR */
    NVIC_EnableIRQ(pdm_pcm_isr_cfg.intrSrc);
}

/* [] END OF FILE */

//function from MFCC was written in main because some of them using #define value

/*static inline float InverseMelScale(float mel_freq){
    return 700.f * (expf(mel_freq / 1127.0f) - 1.0f);
}*/

static inline float MelScale(float freq){
    return 1127.0f * logf(1.0f + freq/700.0f);
}

void init_window_function(float * window_func){     // call this function in main to initialize window_func
    for(int i = 0; i < NFFT ; i++){
        window_func[i] = 0.5 - 0.5*cos( M_2PI * ((float)i) / (NFFT));
    }
}

void create_mel_fbank(float (* mel_fbank)[NFFT/2], int32_t * fbank_filter_first, int32_t * fbank_filter_last){   // create_mel_fbank(mel_fbank,fbank_filter_first, fbank_filter_last);
                                                                                                        // using reference from fbank_filter_first, fbank_filter_last to add new element to array

    int32_t bin, i;

    int32_t num_ffts_bins = NFFT/2;
    float fft_bin_width = ((float)SAMPLE_RATE) / NFFT;
    float mel_low_freq = MelScale(MEL_LOW_FREQ);
    float mel_high_freq = MelScale(MEL_HIGH_FREQ);
    float mel_freq_delta = (mel_high_freq - mel_low_freq) / (NUM_FBANK_BINS + 1);

    for(bin = 0; bin < NUM_FBANK_BINS; bin++){
        float left_mel = mel_low_freq + bin * mel_freq_delta;
        float center_mel = mel_low_freq + (bin + 1) * mel_freq_delta;
        float right_mel = mel_low_freq + (bin + 2) * mel_freq_delta;

        int32_t first_index = -1, last_index = -1;

        for(i = 0; i < num_ffts_bins; i++){
            float freq = (fft_bin_width * i);
            float mel = MelScale(freq);
            mel_fbank[bin][i] = 0.0;

            if(mel > left_mel && mel < right_mel){      // if condition = true, change the element in mel_fbank[bin][i]
                float weight;
                if(mel <= center_mel){
                    weight = (mel - left_mel) / (center_mel - left_mel);
                }
                else{
                    weight = (right_mel - mel) / (right_mel - center_mel);
                }
                mel_fbank[bin][i] = weight;
                if(first_index == -1){
                    first_index = i;
                }
                last_index = i;
            }
        }

        fbank_filter_first[bin] = first_index;
        fbank_filter_last[bin] = last_index;
    }
}

void create_dct_matrix(float * dct_matrix){         // create_dct_matrix(dct_matrix)
                                                    // using reference dect_matrix to initilize dct_matrix
    int32_t k, n;
    float normalizer;
    arm_sqrt_f32(2.0 / (float)NUM_FBANK_BINS, &normalizer);
    for( k = 0; k < N_MFCC; k++){
        for( n = 0; n < NUM_FBANK_BINS; n++){
            dct_matrix[k*NUM_FBANK_BINS + n] = normalizer * cos( ((double)M_PI) / (double)NUM_FBANK_BINS * (n+0.5) * k);
        }
    }
}

// 1 time transform MFCC from NFFT-element input array;
void mfcc_compute(const int16_t * audio_data, q7_t* mfcc_out){         // calculate MFCC, MFCC* here to use reference-element from the init MFCC
    int32_t i, j, bin;

//    TensorFlow way of normalizing .wav data to (-1,1)
    for( i = 0; i < NFFT; i++){
    	frame[i] = (float) audio_data[i] / (float)(1<<15);                             // audio_data[i] / 2^15
    }

//     skipped the fill up process because NFFT = power of 2
//     apply Hanning-Window
    for(i = 0; i < NFFT; i++){
        frame[i] *= window_func[i];
    }
//     Compute FFT with the help of CMSIS-DSP function
    arm_rfft_fast_f32(&rfft, frame, buffer, 0);
//    Covert to power spectrum
//    frame is stored as [real0, realN/2-1, real 1, im1, real2, im2,...]
    int32_t half_dim = NFFT / 2;
    float first_energy = buffer[0] * buffer[0];
    float last_energy = buffer[1] * buffer[1];      // handle this special case
    for(i = 1; i < half_dim; i++){
        float real = buffer[i*2];
        float im = buffer[i*2 + 1];
        buffer[i] = real*real + im*im;
    }
    buffer[0] = first_energy;                       //buffer using element 0 -> half_dim, rest is the computed element from fft
    buffer[half_dim] = last_energy;
    float sqrt_data;
//    Apply mel filterbanks
    for(bin = 0; bin < NUM_FBANK_BINS; bin++){
        float mel_energy = 0;
        int32_t first_index = fbank_filter_first[bin];
        int32_t last_index = fbank_filter_last[bin];
        for( i = first_index; i <= last_index; i++){
            arm_sqrt_f32(buffer[i],&sqrt_data);
            mel_energy += (sqrt_data)*mel_fbank[bin][i];
        }
        mel_energies[bin] = mel_energy;

//        avoid log of zero
        if(mel_energy == 0.0){
            mel_energies[bin] = FLT_MIN;
        }
    }
//    Take log
    for(bin = 0; bin < NUM_FBANK_BINS; bin++){
        mel_energies[bin] = logf(mel_energies[bin]);
    }
//    Take DCT. Uses matrix mul.
    for(i = 0; i < N_MFCC; i++){
        float sum = 0.0;
        for(j = 0; j < NUM_FBANK_BINS; j++){
            sum += dct_matrix[i * NUM_FBANK_BINS + j] * mel_energies[j];
        }
//        Input is Qx.mfcc_dec_bits (from quantization step)
        sum *= (0x1 << mfcc_dec_bits);
        sum = round(sum);
        if(sum >= 127){
            mfcc_out[i] = 127;
        }
        else if(sum <= -128){
            mfcc_out[i] = -128;
        }
        else{
            mfcc_out[i] = sum;
        }
    }
}
// Array MFCC compute
void array_mfcc_compute(arm_status status, int16_t * bigWindow, int16_t * data_in_mfcc, q7_t * data_out_mfcc, int8_t (* array_mfcc)[MFCC_COL]){
    for(uint8_t index = 0; index < MFCC_COL; index++){
        uint16_t index_NFFT = FRAME_STEP*index;
//        Push the element to in-Array to perform RFFT
//        out of MAX_BIG_WINDOW, set the rest element = 0;
        for(uint16_t i = index_NFFT; i < index_NFFT + NFFT; i++){
            if(i < MAX_BIG_WINDOW){
                data_in_mfcc[i - index_NFFT] = bigWindow[i];
            }
            else{
//                Set the Value = 0 when out of range
                data_in_mfcc[i - index_NFFT] = 0.0;
            }
        }
//    	printf("\n\r status = %d, ARM_MATH_SUCCESS = %d ", status_fft, ARM_MATH_SUCCESS);
        if(status == ARM_MATH_SUCCESS)						//ARM_MATH_SUCCESS = 0
            mfcc_compute(data_in_mfcc, data_out_mfcc);

        /*Here need to push data_out into MFCC and
        using MFCC data with the prepared Sample*/
        for(uint8_t mfcc_index = 0; mfcc_index < N_MFCC; mfcc_index++){
            array_mfcc[mfcc_index][index] = data_out_mfcc[mfcc_index];
        }
    }
}

// finding index with the max value in K-NN array
int max_index_int(int *a, int n)
{
  if(n <= 0) return -1;
  int i, max_i = 0;
  int max = a[0];
  for(i = 1; i < n; ++i){
	  if(a[i] > max){
		  max = a[i];
		  max_i = i;
	  }
  }
  return max_i;
}

// finding most appear in K-NN array with n = array-length, element in array < k;
int numToRepeatMax(int* arr1 , int n, int k)
{
    int max_appear[k];
    for(int i = 0; i < k; i++){
    	max_appear[i] = 0;
    }
    for(int i = 0; i < n; i++){
    	max_appear[arr1[i]] += 1;
    }

    int result = max_index_int(max_appear, k);
    return result;
}

int maxAppear(int8_t* arr1 , int n, int k)
{
    int max_appear[k];
    for(int i = 0; i < k; i++){
    	max_appear[i] = 0;
    }
    for(int i = 0; i < n; i++){
    	max_appear[arr1[i]] += 1;
    }

    int result = max_appear[max_index_int(max_appear, k)];
    return result;
}
