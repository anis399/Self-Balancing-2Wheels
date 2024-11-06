/*******************************************************************************
*                      _____ _    _          ____                              *
*                     / ____| |  | |   /\   / __ \                             *
*                    | (___ | |__| |  /  \ | |  | |___                         *
*                     \___ \|  __  | / /\ \| |  | / __|                        *
*                     ____) | |  | |/ ____ \ |__| \__ \                        *
*                    |_____/|_|  |_/_/    \_\___\_\___/                        *
*                                                                              *
********************************************************************************
* All Software (C)
*******************************************************************************/
/*******************************************************************************
*                     _           _           _    _                           *
*         /\         (_)         | |         | |  | |                          *
*        /  \   _ __  _ ___   ___| |__   __ _| | _| | _____  _   _ _ __        *
*       / /\ \ | '_ \| / __| / __| '_ \ / _` | |/ / |/ / _ \| | | | '__|       *
*      / ____ \| | | | \__ \ \__ \ | | | (_| |   <|   < (_) | |_| | |          *
*     /_/    \_\_| |_|_|___/ |___/_| |_|\__,_|_|\_\_|\_\___/ \__,_|_|          *
*                                                                              *
*******************************************************************************/
/*******************************************************************************
 * File
 *
 *  Created on: November 03, 2024
 *  Author: Anis Shakkour
 *  Email:  anis.shakkour399@gmail.com
 *
 *  @brief : Communication out from the MCU. May be Debug or data
 *  @note  :
 *  @todo  :
 *
 * For more information, please refer to the <a href="https://example.com/my_document.pdf">documentation</a>.
*******************************************************************************/
/******************************************************************************
 * Multiple include protection
 *****************************************************************************/
#ifndef APP_H
#define APP_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Macros
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Typedef & Enums
 ******************************************************************************/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*******************************************************************************
 * Interface Functions
 ******************************************************************************/
void app_init(void);

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void);


/*******************************************************************************
 * END
 ******************************************************************************/
#endif  // APP_H



