////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

/*! \file messagesinfo.hpp
 *  \brief Methods to provide message name from message id and vice vera.
 */

//-----------------------------------------------------------------------
// Recursive Inclusion
//-----------------------------------------------------------------------
#ifndef MESSAGESINFO_H
#define MESSAGESINFO_H

//-----------------------------------------------------------------------
// includes
//-----------------------------------------------------------------------
#include "decoders/common/api/env.hpp"
#include <string>
#include "decoders/jsoninterface/api/loaddatafromjson.hpp"

/*! \fn INT GetMessageIDByName(std::string& MessageName)
 * \brief Returns message id from given message name
 * \param [in] MessageName Log name
 * \return Associated Message id
 */
INT GetMessageIDByName(std::string& MessageName);

/*! \fn INT GetMessageNameByID(INT MessageID)
 * \brief Returns Message name from given message name.
 * \param [in] MessageID ID of the decoded log
 * \return Message name string
 */
std::string GetMessageNameByID(INT MessageID);

#endif
