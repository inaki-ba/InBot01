/*************************************************************************************************/
/*!
 *  \file   att_handler.h
 *
 *  \brief  Interface to ATT event handler.
 *
 *          $Date: 2012-03-29 13:24:04 -0700 (Thu, 29 Mar 2012) $
 *          $Revision: 287 $
 *
 *  Copyright (c) 2009 Wicentric, Inc., all rights reserved.
 *  Wicentric confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/
#ifndef ATT_HANDLER_H
#define ATT_HANDLER_H

#include "wsf_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     AttHandlerInit
 *        
 *  \brief  ATT handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for ATT.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AttHandlerInit(wsfHandlerId_t handlerId);


/*************************************************************************************************/
/*!
 *  \fn     AttHandler
 *        
 *  \brief  WSF event handler for ATT.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AttHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

#ifdef __cplusplus
};
#endif

#endif /* ATT_HANDLER_H */
