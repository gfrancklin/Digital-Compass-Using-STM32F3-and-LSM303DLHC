/******************************************************************************  
 *  FILE          : main.c 
 *  PROJECT       : Bare Metal Programming - LAB #3 Magnetometer 
 *  PROGRAMMER    : Gabriel Yano, Artem Bordiuh
 *  REVISED 	  : 07/19/2017
 *  DESCRIPTION   : Added functions for initializing magnetometer as well as
					routine to continiously show North direction 
					by turning on LED which is the closest to the North 
					beside simple monitor commands in the main while looop
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"
#include "stm32f3_discovery_accelerometer.h"
#include "stm32f3_discovery_gyroscope.h"
#include "stdint.h"

#include "Drivers/BSP/Components/lsm303dlhc/lsm303dlhc.h"
#include "common.h"
#include "usertypes.h"
#include "math.h"
#include "stdlib.h"

/* Magnetometer data ---------------------------------------------------------*/
#define CRA_REG_M       0x00
#define CRB_REG_M       0x01
#define MR_REG_M        0x02
#define OUT_X_H_M       0x03
#define OUT_X_L_M       0x04
#define OUT_Z_H_M       0x05
#define OUT_Z_L_M       0x06
#define OUT_Y_H_M       0x07
#define OUT_Y_L_M       0x08
#define SR_REG_M        0x09
#define IRA_REG_M       0x0A
#define IRB_REG_M       0x0B
#define IRC_REG_M       0x0C
#define TEMP_OUT_H_M    0x31
#define TEMP_OUT_L_M    0x32

#define PI              3.14159265
#define NOISY           1.6

/* To store the latest scaled magnetometer XYZ and scaling data*/
COMPDataframe_t cdata;

/* Magnetometer data end -----------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
const Led_TypeDef LEDs[] = {LED3, LED4, LED5, LED6, LED7, LED8, LED9, LED10};
const uint32_t numLEDs = sizeof (LEDs) / sizeof (LEDs[0]);

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct);
void clearAllLeds( void );
void ledCompass( void );

int main(int argc, char **argv) {
    uint32_t i;
    uint8_t accelRc, gyroRc;
    /* Configure the system clock */
    SystemClock_Config();

    HAL_Init();

    TerminalInit(); /* Initialize UART and USB */
    /* Configure the LEDs... */
    for (i = 0; i < numLEDs; i++) {
        BSP_LED_Init(LEDs[i]);
    }

    /* Initialize the pushbutton */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

    /* Initialize the Accelerometer */
    accelRc = BSP_ACCELERO_Init();
    if (accelRc != ACCELERO_OK) {
        printf("Failed to initialize acceleromter\n");
        Error_Handler();
    }

    /* Initialize the Gyroscope */
    gyroRc = BSP_GYRO_Init();
    if (gyroRc != GYRO_OK) {
        printf("Failed to initialize Gyroscope\n");
        Error_Handler();
    }

    my_Init();

    while (1) {
		//Checking if compass with LEDs is On
        if(cdata.state)
            ledCompass();	//turn on LED pointing to North
        TaskInput();
        my_Loop();
        /* Tickle the watchdog */
    }

    return 0;
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV                     = 1
 *            PLLMUL                         = RCC_PLL_MUL9 (9)
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
    BSP_LED_On(LED6);
    /* Infinite loop */
    while (1) {
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
    my_Tick();
}

void CmdLED(int mode) {
    uint32_t led, val;
    int rc;
    if (mode != CMD_INTERACTIVE) {
        return;
    }

    rc = fetch_uint32_arg(&led);
    if (rc) {
        printf("Missing LED index\n");
        return;
    }

    rc = fetch_uint32_arg(&val);
    if (rc) {
        printf("Missing state value, 0 for Off, 1 for On\n");
        return;
    }

    if ((led < 3) || (led > 10)) {
        printf("Led index of %u is out of the range (3..10)\n",
                (unsigned int) led);
        return;
    }

    led -= 3;
    if (val) {
        BSP_LED_On(LEDs[led]);
    } else {
        BSP_LED_Off(LEDs[led]);
    }

}

ADD_CMD("led", CmdLED, "<index> <state> Turn off/on LED")

void CmdAccel(int mode) {
    int16_t xyz[3];

    if (mode != CMD_INTERACTIVE) {
        return;
    }

    BSP_ACCELERO_GetXYZ(xyz);

    printf("Accelerometer returns:\n"
            "   X: %d\n"
            "   Y: %d\n"
            "   Z: %d\n",
            xyz[0], xyz[1], xyz[2]);


}

ADD_CMD("accel", CmdAccel, "                Read Accelerometer");

void CmdGyro(int mode) {
    float xyz[3];

    if (mode != CMD_INTERACTIVE) {
        return;
    }

    BSP_GYRO_GetXYZ(xyz);

    printf("Gyroscope returns:\n"
            "   X: %d\n"
            "   Y: %d\n"
            "   Z: %d\n",
            (int) (xyz[0]*256),
            (int) (xyz[1]*256),
            (int) (xyz[2]*256));
}

ADD_CMD("gyro", CmdGyro, "                Read Gyroscope");


void CmdButton(int mode) {
    uint32_t button;

    if (mode != CMD_INTERACTIVE) {
        return;
    }

    button = BSP_PB_GetState(BUTTON_USER);

    printf("Button is currently: %s\n",
            button ? "Pressed" : "Released");

    return;
}

ADD_CMD("button", CmdButton, " Print status of user button");

/**************************************************************************************************
*	@brief	Read Magnetometer XYZ data (should be initialized previously). Will turn the LED which 
*			is pointing to the North
*	@param None
*	@return None	
**************************************************************************************************/
void ledCompass(){	
    /* Getting data */
    cdata.raw[0] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, OUT_X_H_M);							//X High
    cdata.raw[0] = (cdata.raw[0] << 8) + COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, OUT_X_L_M);	//X Low
    cdata.raw[1] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, OUT_Y_H_M);							//Y High
    cdata.raw[1] = (cdata.raw[1] << 8) + COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, OUT_Y_L_M);	//Y Low
    cdata.raw[2] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, OUT_Z_H_M);							//Z High
    cdata.raw[2] = (cdata.raw[2] << 8) + COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, OUT_Z_L_M);	//Z Low

	/* Scaling the compass data from raw values */
    for (int ii = 0; ii < COMP_AXES; ii++) {
        if (ii < COMP_AXES - 1)
            cdata.scaled[ii] = ((float) cdata.raw[ii]) / cdata.sensitivity_xy;
        else
            cdata.scaled[ii] = ((float) cdata.raw[ii]) / cdata.sensitivity_z;
    }
	
     /* Calculating "error" and theta on XY plane */
    float error = sqrt((cdata.scaled[0] * cdata.scaled[0]) +
            (cdata.scaled[1] * cdata.scaled[1]));
    float theta = atan(cdata.scaled[1] / cdata.scaled[0]) * 180 / PI;
    
    clearAllLeds();
    
	/* Checking if compass is not plane and if the noise is high */
    if (abs(error) > abs(NOISY * cdata.scaled[2])) {
		/* Glowing 4 LEDs meaning environment to noisy or board not plane */
        BSP_LED_On(LED7);
        BSP_LED_On(LED6);
        BSP_LED_On(LED3);
        BSP_LED_On(LED10);        
    } else {
       /* Otherwise turn on LED pointing to North */
        if (cdata.scaled[0] > 0) {		//The North is in 3d and 4th quadrant
            if (theta < -67.5)
                BSP_LED_On(LED6);
            if ((theta > -67.5) && (theta < -22.5))
                BSP_LED_On(LED8);
            if ((theta > -22.5) && (theta < 22.5))
                BSP_LED_On(LED10);
            if ((theta > 22.5) && (theta < 67.5))
                BSP_LED_On(LED9);
            if (theta > 67.5)
                BSP_LED_On(LED7);
        } else {						//The North is in 1d and 2d quadrant
            if (theta < -67.5)
                BSP_LED_On(LED7);
            if ((theta > -67.5) && (theta < -22.5))
                BSP_LED_On(LED5);
            if ((theta > -22.5) && (theta < 22.5))
                BSP_LED_On(LED3);
            if ((theta > 22.5) && (theta < 67.5))
                BSP_LED_On(LED4);
            if (theta > 67.5)
                BSP_LED_On(LED6);
        }
    }
}

/**************************************************************************************************
*	@brief	Switch state wariable so main while(1) loop will continiously read the magnetometer
			and point to the North by exact LED
			1..* - is On
			0 - is Off
*	@return None	
**************************************************************************************************/
void turnOnMagnetometer(int mode){
    if (mode != CMD_INTERACTIVE) {
        return;
    }
    
    int32_t state;
    if (fetch_int32_arg(&state)) {
        printf("Wrong state\n");
        return;
    }
    cdata.state = (state>0)? 1 : 0;
    clearAllLeds();
};
ADD_CMD("compass", turnOnMagnetometer, "<state>         Activate LED compass");

/**************************************************************************************************
*	@brief	Initializing magnetometer for continuos conversation of its values. 
*			Sensetivity	is 2 5 GA for XYZ
*			Setting the scaling variables
*	@return None	
**************************************************************************************************/
void magnetInit(int mode) {
    if (mode != CMD_INTERACTIVE) {
        return;
    }

    /* Write value to Mag MEMS CRA_REG regsister */
    COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, LSM303DLHC_ODR_30_HZ);
    /* Write value to Mag MEMS CRB_REG regsister */
    COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, LSM303DLHC_FS_2_5_GA);
    /* Write value to Mag MEMS MR_REG regsister */
    COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, LSM303DLHC_CONTINUOS_CONVERSION);
	
	/* Setting the initial values to calculate magnetometer output */
    cdata.full_scale = 2.5;
    cdata.sensitivity_xy = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    cdata.sensitivity_z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;

    printf("Status is: %d\n", LSM303DLHC_MagGetDataStatus());
}
ADD_CMD("mi", magnetInit, "Initialize Magnet");
