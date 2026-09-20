/****************************************************************************
 * Keystore slots this library owns. A board that has already spoken for one
 * of these overrides it in its board config.
 ****************************************************************************/

#pragma once

#ifndef ZTCS_KEY_SLOT_STATION_PUBLIC
#define ZTCS_KEY_SLOT_STATION_PUBLIC 3
#endif
#ifndef ZTCS_KEY_SLOT_OPERATOR_PUBLIC
#define ZTCS_KEY_SLOT_OPERATOR_PUBLIC 4
#endif
#ifndef ZTCS_KEY_SLOT_LINK
#define ZTCS_KEY_SLOT_LINK 15
#endif
#ifndef ZTCS_KEY_SLOT_IDENTITY
#define ZTCS_KEY_SLOT_IDENTITY 17
#endif
