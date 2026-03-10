#ifndef INC_RAMN_J1939_H_
#define INC_RAMN_J1939_H_

/* J1939 PGN Definitions */
#define J1939_PGN_TSC1              0       /* Torque/Speed Control 1 */
#define J1939_PGN_TC1               256     /* Transmission Control 1 */
#define J1939_PGN_EBS1              512     /* Electronic Brake System 1 */
#define J1939_PGN_XBR               1024    /* External Brake Request */
#define J1939_PGN_PROPA             61184   /* Proprietary A (PDU1) */
#define J1939_PGN_EBC1              61441   /* Electronic Brake Controller 1 */
#define J1939_PGN_EEC2              61443   /* Electronic Engine Controller 2 */
#define J1939_PGN_EEC1              61444   /* Electronic Engine Controller 1 */
#define J1939_PGN_ETC2              61445   /* Electronic Transmission Controller 2 */
#define J1939_PGN_ESC1              61451   /* Electronic Steering Control 1 */
#define J1939_PGN_ENGINE_KEY        64960   /* PGN for Engine Key / Ignition */
#define J1939_PGN_OEL               64972   /* Operators External Light Controls */
#define J1939_PGN_CM3               64980   /* Cab Message 3 */
#define J1939_PGN_LIGHTS_CMD        65089   /* Lighting Command */
#define J1939_PGN_HORN_STATUS       65098   /* Secondary Air / Horn Status */
#define J1939_PGN_CCVS1             65265   /* Cruise Control/Vehicle Speed 1 */
#define J1939_PGN_DM1               65226   /* DM1 - Active Diagnostic Trouble Codes */

/* J1939 Logical Source Addresses (SA) */
#define J1939_SA_ENGINE             0
#define J1939_SA_TRANSMISSION       3
#define J1939_SA_SHIFT_CONSOLE      5
#define J1939_SA_BRAKE_SYSTEM       11
#define J1939_SA_STEERING_CTRL      19
#define J1939_SA_BODY_CTRL          33
#define J1939_SA_STEERING_COLUMN    77
#define J1939_SA_AEBS               160

/* J1939 Destination Addresses (DA) */
#define J1939_DA_ENGINE             0
#define J1939_DA_TRANSMISSION       3
#define J1939_DA_BRAKE_SYSTEM       11
#define J1939_DA_STEERING_CTRL      19
#define J1939_DA_BODY_CTRL          33
#define J1939_DA_BROADCAST          255

/* Proprietary A Payload Definitions */
/* Command_Steering (PGN 61184): SA AEBS (160), DA Steering Controller (19). Bytes 1-2. j1939_val = ramn_val */
/* Command_Horn (PGN 61184): SA Shift Console (5), DA Body Controller (33). Byte 1. j1939_val = ramn_val & 0xFF */

#endif /* INC_RAMN_J1939_H_ */
