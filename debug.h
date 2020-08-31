#ifndef KRB_DEBUG_H
#define KRB_DEBUG_H


#define DEBUG_INFO
#define DEBUG_INFO_PLUS
//#define DEBUG_INFO_PLUS_PLUS


#ifdef DEBUG_INFO
    #define INFO(x) x
#else
    #define INFO(x)
#endif

#ifdef DEBUG_INFO_PLUS
    #define INFO_P(x) x
#else
    #define INFO_P(x)
#endif

#ifdef DEBUG_INFO_PLUS_PLUS
    #define INFO_PP(x) x
#else
    #define INFO_PP(x)
#endif


#endif