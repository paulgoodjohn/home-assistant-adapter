/*!
 * @file
 * @brief Erd list access for various appliances
 */

#ifndef APPLIANCEERDS_H
#define APPLIANCEERDS_H

#include "tiny_erd.h"

typedef struct
{
  const tiny_erd_t* erdList;
  const uint16_t erdCount;
} tiny_erd_list_t;

/*!
 * Get the list of common ERDs
 */
const tiny_erd_list_t* GetCommonErdList(void);

/*!
 * Get the list of energy ERDs
 */
const tiny_erd_list_t* GetEnergyErdList(void);

/*!
 * Get the list of appliance ERDs based on appliance type
 */
const tiny_erd_list_t* GetApplianceErdList(uint8_t applianceType);

#endif
