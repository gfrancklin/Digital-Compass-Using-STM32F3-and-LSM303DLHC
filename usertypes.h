/*
    Pretty LAYer for ChibiOS/RT - Copyright (C) 2015 Rocco Marco Guglielmi

    This file is part of PLAY for ChibiOS/RT.

    PLAY is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    PLAY is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
    Special thanks to Giovanni Di Sirio for teachings, his moral support and
    friendship. Note that some or every piece of this file could be part of
    the ChibiOS project that is intellectual property of Giovanni Di Sirio.
    Please refer to ChibiOS/RT license before use this file.

    For suggestion or Bug report - guglielmir@playembedded.org
 */

/**
 * @file    usertypes.h
 * @brief   Types extracted from PLAY lib.
 *
 * @{
 */

#ifndef _USERTYPES_H_
#define _USERTYPES_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Gyro number of axes.
 */
#define GYRO_AXES                                0x03
/**
 * @brief   Accel number of axes.
 */
#define ACCEL_AXES                               0x03
/**
 * @brief   Compass number of axes.
 */
#define COMP_AXES                                0x03

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   GYRO default data type
 * @details According to L3GD20 axis data is a signed integer long 16 bit
 */
typedef int16_t         GYROdata_t;


/**
 * @brief   GYRO advanced data type
 * @details This axis data is to be considered as degrees per seconds
 */
typedef float           GYROfdata_t;

/**
 * @brief   GYRO data frame
 */
typedef struct{
  /**
   * @brief axes data as integer long 16 bits
   */
  GYROdata_t raw[GYRO_AXES];

  /**
   * @brief full scale value
   */
  GYROfdata_t full_scale;
  /**
   * @brief sensitivity value
   */
  GYROfdata_t sensitivity;
  /**
   * @brief   axis data as floating point
   * @details This axis data is to be considered as degrees per seconds
   */
  GYROfdata_t scaled[GYRO_AXES];
} GYRODataframe_t;

/**
 * @brief   ACCEL default data type
 * @details According to LSM303DLHC axis data is a signed integer long 16 bit
 */
typedef int16_t         ACCELdata_t;

/**
 * @brief   ACCEL advanced data type
 * @details This axis data is to be considered as meter per seconds
 */
typedef float           ACCELfdata_t;

/**
 * @brief   ACCEL data frame
 */
typedef struct {
  /**
   * @brief axes data as integer long 16 bits
   */
  ACCELdata_t raw[ACCEL_AXES];
  /**
   * @brief full scale value
   */
  ACCELfdata_t full_scale;
  /**
   * @brief sensitivity value
   */
  ACCELfdata_t sensitivity;
  /**
   * @brief   axes data as floating point
   * @details This axis data is to be considered as  meter per square seconds
   */
  ACCELfdata_t scaled[ACCEL_AXES];
} ACCELDataframe_t;

/**
 * @brief   COMP default data type
 * @details According to LSM303DLHC axis data is a signed integer long 16 bit
 */
typedef int16_t         COMPdata_t;

/**
 * @brief   COMP advanced data type
 * @details This axis data is to be considered as Gauss
 */
typedef float           COMPfdata_t;

/**
 * @brief   COMPASS data frame
 */
typedef struct {
  /**
   * @brief axes data as integer long 16 bits
   */
  COMPdata_t raw[COMP_AXES];

  /**
   * @brief full scale value
   */
  COMPfdata_t full_scale;
  /**
   * @brief sensitivity value for xy plane
   */
  COMPfdata_t sensitivity_xy;
  /**
   * @brief sensitivity value for z axis
   */
  COMPfdata_t sensitivity_z;
  /**
   * @brief   axes data as floating point
   * @details This axis data is to be considered as Gauss.
   */
  COMPfdata_t scaled[COMP_AXES];
  
  /*
   * @brief On/Off state
   */
  int8_t state;
} COMPDataframe_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* _USERTYPES_H_ */

/** @} */