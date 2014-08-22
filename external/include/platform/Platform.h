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
 * @file Platform.h
 *    Platform dependent defines and macros.
 *
 * @author Tim Langner
 * @date   2010/07/07
 */

#ifndef _MIRA_PLATFORM_H_
#define _MIRA_PLATFORM_H_

///////////////////////////////////////////////////////////////////////////////

/**
 * The namespace mira is the top level namespace for all core and framework
 * classes and functions as well as gui components. Domains and Toolboxes must use
 * their own or sub namespaces to avoid ambiguities of names.
 */
namespace mira {}

///////////////////////////////////////////////////////////////////////////////

#undef MIRA_LINUX
#undef MIRA_WINDOWS

// POSIX standard for gcc
#ifdef __linux__
#  define MIRA_LINUX
#else
#  define MIRA_WINDOWS
#endif

// Architecture type
#if __WORDSIZE == 64
# define MIRA_ARCH64
#else
# define MIRA_ARCH32
#endif

// defines for FUNCTION macro
#ifdef MIRA_LINUX
#  define MIRA_FUNCTION __PRETTY_FUNCTION__
#else
#  define MIRA_FUNCTION __FUNCTION__
#endif

#ifdef MIRA_LINUX
#  define MIRA_DEFAULT_CONSTRUCTOR =default;
#else
#  define MIRA_DEFAULT_CONSTRUCTOR {}
#endif

// Some Windows specific settings
#ifdef MIRA_WINDOWS
#  ifdef _MSC_VER
#    // Disable some warnings in context of DLL exports/imports
#    pragma warning(disable : 4251)
#    pragma warning(disable : 4275)
#
#    // Disable some more warnings on Windows
#    pragma warning(disable : 4355)
#
#    // Disable "conversion from 'size_t' to 'int', possible loss of data"
#    pragma warning(disable : 4267)
#    // Disable "conversion from '__int64' to 'int', possible loss of data"
#    pragma warning(disable : 4244)
#  endif
#
#  ifndef WIN32_LEAN_AND_MEAN
#    // Avoid include old or unusual used Windows heads (like old Winsock.h)
#    define WIN32_LEAN_AND_MEAN 1
#  endif
#  include <windows.h>
#
#  /// This is required because on windows there is a macro defined called ERROR
#  ifdef ERROR
#    undef ERROR
#  endif
#
#  /// This is required because on windows there is a macro defined called RGB
#  ifdef RGB
#    undef RGB
#  endif
#
#  /// The Microsoft Compilers use '#define interface struct'. Since this is
#  /// not really nice and 'interface' is not a reserved keyword in C/C++, we
#  /// decided to undef it here!
#  ifdef interface
#     undef interface
#  endif
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef MIRA_WINDOWS
#  ifdef MIRA_BASE_EXPORTS
#    define MIRA_BASE_EXPORT __declspec(dllexport)
#  else
#    define MIRA_BASE_EXPORT __declspec(dllimport)
#  endif
#else
#  define MIRA_BASE_EXPORT
#endif

///////////////////////////////////////////////////////////////////////////////

/**
 * Macro for declaring functions and classes deprecated.
 * \code
 * MIRA_DEPRECATED("Please use myNewFunction instead", void myOldFunction()) {}
 * \endcode
 */
#ifdef MIRA_WINDOWS
# define MIRA_DEPRECATED(text, decl) __declspec(deprecated(text)) decl
# define MIRA_DEPRECATED_CLASS(text) __declspec(deprecated(text))
#else
# define MIRA_DEPRECATED(text, decl) decl __attribute__ ((deprecated))
# define MIRA_DEPRECATED_CLASS(text) __attribute__ ((deprecated))
#endif

//////////////////////////////////////////////////////////////////////////////

/**
 * The following macro MIRA_GNUC_VERSION combines the gcc compiler version numbers
 * in a single number.
 * One can test if the compiler meets a required version e.g. 4.6.1 with:
 * #if MIRA_GNUC_VERSION == 40601
 */
#if defined(__GNUC__)
# if defined(__GNUC_PATCHLEVEL__)
#  define MIRA_GNUC_VERSION (__GNUC__ * 10000 \
                            + __GNUC_MINOR__ * 100 \
                            + __GNUC_PATCHLEVEL__)
# else
#  define MIRA_GNUC_VERSION (__GNUC__ * 10000 \
                            + __GNUC_MINOR__ * 100)
# endif
#endif

///////////////////////////////////////////////////////////////////////////////

#endif
