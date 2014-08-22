/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file Types.h
 *    Typedefs for OS independent basic data types.
 *
 * @author Tim Langner
 * @date   2010/07/14
 */

#ifndef _MIRA_TYPES_H_
#define _MIRA_TYPES_H_

#include <platform/Platform.h>

#include <cstddef>
#include <string.h>
#include <assert.h>

#ifdef MIRA_LINUX

# include <stdint.h>

typedef int8_t                int8;
typedef int16_t               int16;
typedef int32_t               int32;
typedef int64_t               int64;
typedef uint8_t               uint8;
typedef uint16_t              uint16;
typedef uint32_t              uint32;
typedef uint64_t              uint64;

#elif defined(MIRA_WINDOWS)

typedef signed __int8         int8;
typedef signed __int16        int16;
typedef signed __int32        int32;
typedef signed __int64        int64;
typedef unsigned __int8       uint8;
typedef unsigned __int16      uint16;
typedef unsigned __int32      uint32;
typedef unsigned __int64      uint64;

#endif

#endif
