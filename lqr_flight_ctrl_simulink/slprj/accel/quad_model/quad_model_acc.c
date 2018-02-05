#include "__cf_quad_model.h"
#include <math.h>
#include "quad_model_acc.h"
#include "quad_model_acc_private.h"
#include <stdio.h>
#include "slexec_vm_simstruct_bridge.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_lookup_functions.h"
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
#include "simtarget/slAccSfcnBridge.h"
static void mdlOutputs ( SimStruct * S , int_T tid ) { real_T nxy3eg5532 ;
a0uytdxicy * _rtB ; oiicjvkw1o * _rtP ; plyhzewzek * _rtX ; h0exlopj4l *
_rtDW ; _rtDW = ( ( h0exlopj4l * ) ssGetRootDWork ( S ) ) ; _rtX = ( (
plyhzewzek * ) ssGetContStates ( S ) ) ; _rtP = ( ( oiicjvkw1o * )
ssGetModelRtp ( S ) ) ; _rtB = ( ( a0uytdxicy * ) _ssGetModelBlockIO ( S ) )
; ssCallAccelRunBlock ( S , 0 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 0 , 1 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock
( S , 0 , 2 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 0 , 3 ,
SS_CALL_MDL_OUTPUTS ) ; _rtB -> owgb1yuysh = _rtB -> bxdrnegtoi - _rtB ->
cznqd5zbwb [ 2 ] ; _rtB -> ho0rewziyn = _rtP -> P_7 * _rtB -> owgb1yuysh ;
_rtB -> o0zrpaqjdv = _rtX -> eclxswx3o1 ; _rtB -> ank50oezyc = _rtB ->
ho0rewziyn - _rtB -> o0zrpaqjdv ; _rtB -> lgcy3zqxsl = _rtP -> P_9 * _rtB ->
ank50oezyc ; _rtB -> he31zhvfh5 = _rtB -> px0hd4k455 - _rtB -> cznqd5zbwb [ 2
] ; _rtB -> nwn2xiz5on = _rtP -> P_10 * _rtB -> he31zhvfh5 ; _rtB ->
l1njg0t3nv = _rtB -> hsel5hk0il - _rtB -> cznqd5zbwb [ 1 ] ; _rtB ->
gpe1kbqknz = _rtP -> P_13 * _rtB -> l1njg0t3nv ; _rtB -> cutb3yfjrl = _rtX ->
gxunomsa10 ; _rtB -> pz40tg5r0f = _rtB -> gpe1kbqknz - _rtB -> cutb3yfjrl ;
_rtB -> l33cj2xcca = _rtP -> P_15 * _rtB -> pz40tg5r0f ; _rtB -> l0emwibw4t =
_rtB -> cabymyrrjm - _rtB -> cznqd5zbwb [ 1 ] ; _rtB -> gtrrvcoicm = _rtP ->
P_16 * _rtB -> l0emwibw4t ; _rtB -> gvqy5mr2uf = _rtX -> b2q1wtfuy0 ; _rtB ->
eo2x4nsv20 = _rtB -> p4isa4rgnj - _rtB -> cznqd5zbwb [ 1 ] ; _rtB ->
ooxp4z00u4 = _rtP -> P_19 * _rtB -> eo2x4nsv20 ; _rtB -> lelzumoqap = ( _rtB
-> ooxp4z00u4 + _rtB -> gvqy5mr2uf ) + _rtB -> l33cj2xcca ; _rtB ->
bovjwjw5sa = _rtB -> pk5txb3hwu - _rtB -> cznqd5zbwb [ 0 ] ; _rtB ->
oc0t4ehpsd = _rtP -> P_21 * _rtB -> bovjwjw5sa ; _rtB -> oy0gk10unm = _rtX ->
mac20sf5xm ; _rtB -> nm5kaj5tor = _rtB -> oc0t4ehpsd - _rtB -> oy0gk10unm ;
_rtB -> axr4nx2ez3 = _rtP -> P_23 * _rtB -> nm5kaj5tor ; _rtB -> couvdiuhz5 =
_rtB -> jqjy1zppkh - _rtB -> cznqd5zbwb [ 0 ] ; _rtB -> jdfyjkbaj3 = _rtP ->
P_24 * _rtB -> couvdiuhz5 ; _rtB -> po5xjpcizv = _rtX -> c1lnotgjhi ; _rtB ->
nxyezqsvrv = _rtB -> oe1azohv5j - _rtB -> cznqd5zbwb [ 0 ] ; _rtB ->
lcgxqfwyh2 = _rtP -> P_27 * _rtB -> nxyezqsvrv ; _rtB -> f42lpxazvd = ( _rtB
-> lcgxqfwyh2 + _rtB -> po5xjpcizv ) + _rtB -> axr4nx2ez3 ; nxy3eg5532 =
muDoubleScalarCos ( _rtB -> cznqd5zbwb [ 1 ] ) ; _rtB -> fqhnbgopcg =
nxy3eg5532 * muDoubleScalarSin ( _rtB -> cznqd5zbwb [ 0 ] ) ; _rtB ->
kjcjvilt2f = _rtB -> fqhnbgopcg * _rtB -> guch2tm4n1 * _rtB -> llrx10h42m *
_rtB -> diyg1yxkrx ; _rtB -> a44hdksd1g = muDoubleScalarSin ( _rtB ->
cznqd5zbwb [ 1 ] ) * _rtB -> guch2tm4n1 ; _rtB -> ap0ssexlxq = _rtB ->
llrx10h42m * muDoubleScalarCos ( _rtB -> cznqd5zbwb [ 0 ] ) * nxy3eg5532 *
_rtB -> guch2tm4n1 ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { ssCallAccelRunBlock
( S , 0 , 75 , SS_CALL_MDL_OUTPUTS ) ; } _rtB -> bjjvvqckz0 [ 0 ] = _rtB ->
kjcjvilt2f ; _rtB -> bjjvvqckz0 [ 1 ] = 0.0 ; _rtB -> bjjvvqckz0 [ 2 ] = 0.0
; _rtB -> bjjvvqckz0 [ 3 ] = 0.0 ; _rtB -> c5ggow14m5 [ 0 ] = _rtB ->
a44hdksd1g ; _rtB -> c5ggow14m5 [ 1 ] = 0.0 ; _rtB -> c5ggow14m5 [ 2 ] = 0.0
; _rtB -> c5ggow14m5 [ 3 ] = 0.0 ; _rtB -> oquf1sfcf3 [ 0 ] = _rtB ->
ap0ssexlxq ; _rtB -> oquf1sfcf3 [ 1 ] = 0.0 ; _rtB -> oquf1sfcf3 [ 2 ] = 0.0
; _rtB -> oquf1sfcf3 [ 3 ] = 0.0 ; _rtB -> polb14g2ui [ 0 ] = _rtB ->
f42lpxazvd ; _rtB -> polb14g2ui [ 1 ] = 0.0 ; _rtB -> polb14g2ui [ 2 ] = 0.0
; _rtB -> polb14g2ui [ 3 ] = 0.0 ; _rtB -> gcnu5e4asd [ 0 ] = _rtB ->
lelzumoqap ; _rtB -> gcnu5e4asd [ 1 ] = 0.0 ; _rtB -> gcnu5e4asd [ 2 ] = 0.0
; _rtB -> gcnu5e4asd [ 3 ] = 0.0 ; _rtB -> ncg1a5ukpm [ 0 ] = 0.0 ; _rtB ->
ncg1a5ukpm [ 1 ] = 0.0 ; _rtB -> ncg1a5ukpm [ 2 ] = 0.0 ; _rtB -> ncg1a5ukpm
[ 3 ] = 0.0 ; ssCallAccelRunBlock ( S , 0 , 82 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 0 , 83 , SS_CALL_MDL_OUTPUTS ) ; UNUSED_PARAMETER (
tid ) ; } static void mdlOutputsTID2 ( SimStruct * S , int_T tid ) {
a0uytdxicy * _rtB ; oiicjvkw1o * _rtP ; _rtP = ( ( oiicjvkw1o * )
ssGetModelRtp ( S ) ) ; _rtB = ( ( a0uytdxicy * ) _ssGetModelBlockIO ( S ) )
; _rtB -> llrx10h42m = _rtP -> P_0 ; _rtB -> px0hd4k455 = _rtP -> P_1 ; _rtB
-> diyg1yxkrx = _rtP -> P_2 ; _rtB -> guch2tm4n1 = _rtP -> P_3 ; _rtB ->
jqjy1zppkh = _rtP -> P_4 ; _rtB -> cabymyrrjm = _rtP -> P_5 ; _rtB ->
bxdrnegtoi = _rtP -> P_6 * _rtB -> px0hd4k455 ; _rtB -> hsel5hk0il = _rtP ->
P_12 * _rtB -> cabymyrrjm ; _rtB -> p4isa4rgnj = _rtP -> P_18 * _rtB ->
cabymyrrjm ; _rtB -> pk5txb3hwu = _rtP -> P_20 * _rtB -> jqjy1zppkh ; _rtB ->
oe1azohv5j = _rtP -> P_26 * _rtB -> jqjy1zppkh ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { UNUSED_PARAMETER ( tid
) ; }
#define MDL_UPDATE
static void mdlUpdateTID2 ( SimStruct * S , int_T tid ) { UNUSED_PARAMETER (
tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { a0uytdxicy * _rtB ; m04ohyevhh
* _rtXdot ; _rtXdot = ( ( m04ohyevhh * ) ssGetdX ( S ) ) ; _rtB = ( (
a0uytdxicy * ) _ssGetModelBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 0 , 0 ,
SS_CALL_MDL_DERIVATIVES ) ; _rtXdot -> eclxswx3o1 = _rtB -> lgcy3zqxsl ;
_rtXdot -> nrtkru04fn = _rtB -> nwn2xiz5on ; _rtXdot -> gxunomsa10 = _rtB ->
l33cj2xcca ; _rtXdot -> b2q1wtfuy0 = _rtB -> gtrrvcoicm ; _rtXdot ->
mac20sf5xm = _rtB -> axr4nx2ez3 ; _rtXdot -> c1lnotgjhi = _rtB -> jdfyjkbaj3
; }
#define MDL_PROJECTION
static void mdlProjection ( SimStruct * S ) { ssCallAccelRunBlock ( S , 0 , 0
, SS_CALL_MDL_PROJECTION ) ; } static void mdlInitializeSizes ( SimStruct * S
) { ssSetChecksumVal ( S , 0 , 2032372238U ) ; ssSetChecksumVal ( S , 1 ,
2365797044U ) ; ssSetChecksumVal ( S , 2 , 3786901455U ) ; ssSetChecksumVal (
S , 3 , 3298408820U ) ; { mxArray * slVerStructMat = NULL ; mxArray *
slStrMat = mxCreateString ( "simulink" ) ; char slVerChar [ 10 ] ; int status
= mexCallMATLAB ( 1 , & slVerStructMat , 1 , & slStrMat , "ver" ) ; if (
status == 0 ) { mxArray * slVerMat = mxGetField ( slVerStructMat , 0 ,
"Version" ) ; if ( slVerMat == NULL ) { status = 1 ; } else { status =
mxGetString ( slVerMat , slVerChar , 10 ) ; } } mxDestroyArray ( slStrMat ) ;
mxDestroyArray ( slVerStructMat ) ; if ( ( status == 1 ) || ( strcmp (
slVerChar , "8.7" ) != 0 ) ) { return ; } } ssSetOptions ( S ,
SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork ( S ) != sizeof (
h0exlopj4l ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( a0uytdxicy ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
oiicjvkw1o ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetModelRtp ( S , ( real_T *
) & oclf0xym2o ) ; } static void mdlInitializeSampleTimes ( SimStruct * S ) {
slAccRegPrmChangeFcn ( S , mdlOutputsTID2 ) ; } static void mdlTerminate (
SimStruct * S ) { }
#include "simulink.c"
