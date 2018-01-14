/*
 * HC_RemoteCodes
 * Version 0.1 April, 2015
 *
 * Set of constants for the IR remotes used at HedgeCourt.
 */

#ifndef HC_RemoteCodes_h
#define HC_RemoteCodes_h

#ifdef HC_REMOTE_SPECIAL_FOR_MP3

#define HC_RC_POWER 0xFFA25D
#define HC_RC_MODE 0xFF629D
#define HC_RC_MUTE 0xFFE21D
#define HC_RC_PLAYPAUSE 0xFF22DD
#define HC_RC_PAGE_BACK 0xFF02FD
#define HC_RC_PAGE_FORWARD 0xFFC23D
#define HC_RC_EQ 0xFFE01F
#define HC_RC_VOL_DOWN 0xFFA857
#define HC_RC_VOL_UP 0xFF906F
#define HC_RC_SQUIGGLE 0xFF9867
#define HC_RC_USD 0xFFB04F
#define HC_RC_0 0xFF6897
#define HC_RC_1 0xFF30CF
#define HC_RC_2 0xFF18E7
#define HC_RC_3 0xFF7A85
#define HC_RC_4 0xFF10EF
#define HC_RC_5 0xFF38C7
#define HC_RC_6 0xFF5AA5
#define HC_RC_7 0xFF42BD
#define HC_RC_8 0xFF4AB5
#define HC_RC_9 0xFF52AD


#else

// YOU DIDNT DEFINE HC_REMOTE_xxxx so these values are probably garbage
#define HC_RC_POWER 0xFFA25D
#define HC_RC_MODE 0xFF629D
#define HC_RC_MUTE 0xFFE21D
#define HC_RC_PLAYPAUSE 0xFF22DD
#define HC_RC_PAGE_BACK 0xFF02FD
#define HC_RC_PAGE_FORWARD 0xFFC23D
#define HC_RC_EQ 0xFFE01F
#define HC_RC_VOL_DOWN 0xFFA857
#define HC_RC_VOL_UP 0xFF906F
#define HC_RC_SQUIGGLE 0xFF9867
#define HC_RC_USD 0xFFB04F
#define HC_RC_0 0xFF6897
#define HC_RC_1 0xFF30CF
#define HC_RC_2 0xFF18E7
#define HC_RC_3 0xFF7A85
#define HC_RC_4 0xFF10EF
#define HC_RC_5 0xFF38C7
#define HC_RC_6 0xFF5AA5
#define HC_RC_7 0xFF42BD
#define HC_RC_8 0xFF4AB5
#define HC_RC_9 0xFF52AD	

#endif

#endif
