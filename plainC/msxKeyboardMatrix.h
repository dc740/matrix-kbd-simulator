#pragma once

#define TO_COLUMN_ROW(x, y) (((x << 4) & 0xF0) | (y & 0xF))
#define COLUMN_FROM_EVENT(event) ((event & 0xF0) >> 4)
#define ROW_FROM_EVENT(event) (event & 0xF)


// Y0..8
#define column0  0
#define column1  1 << 4
#define column2  2 << 4
#define column3  3 << 4
#define column4  4 << 4
#define column5  5 << 4
#define column6  6 << 4
#define column7  7 << 4
#define column8  8 << 4

#define _NONE          0x80 //invalid key for matrix

#define _0            ( column0 | 0 )
#define _1            ( column0 | 1 )
#define _2            ( column0 | 2 )
#define _3            ( column0 | 3 )
#define _4            ( column0 | 4 )
#define _5            ( column0 | 5 )
#define _6            ( column0 | 6 )
#define _7            ( column0 | 7 )

#define _8            ( column1 | 0 )
#define _9            ( column1 | 1 )
#define _MINUS        ( column1 | 2 )
#define _EQUAL        ( column1 | 3 )
#define _BACKSLASH    ( column1 | 4 )
#define _OPENBRACKET  ( column1 | 5 )
#define _CLOSEBRACKET ( column1 | 6 )
#define _N_TILDE      ( column1 | 7 ) // ñ

#define _APOSTROPHE   ( column2 | 0 )
#define _SEMICOLON    ( column2 | 1 )
#define _COMMA        ( column2 | 2 )
#define _DOT          ( column2 | 3 )
#define _SLASH        ( column2 | 4 )
#define _ACUTE        ( column2 | 5 ) // ´
#define _A            ( column2 | 6 )
#define _B            ( column2 | 7 )

#define _C            ( column3 | 0 )
#define _D            ( column3 | 1 )
#define _E            ( column3 | 2 )
#define _F            ( column3 | 3 )
#define _G            ( column3 | 4 )
#define _H            ( column3 | 5 )
#define _I            ( column3 | 6 )
#define _J            ( column3 | 7 )

#define _K            ( column4 | 0 )
#define _L            ( column4 | 1 )
#define _M            ( column4 | 2 )
#define _N            ( column4 | 3 )
#define _O            ( column4 | 4 )
#define _P            ( column4 | 5 )
#define _Q            ( column4 | 6 )
#define _R            ( column4 | 7 )

#define _S            ( column5 | 0 )
#define _T            ( column5 | 1 )
#define _U            ( column5 | 2 )
#define _V            ( column5 | 3 )
#define _W            ( column5 | 4 )
#define _X            ( column5 | 5 )
#define _Y            ( column5 | 6 )
#define _Z            ( column5 | 7 )

#define _SHIFT        ( column6 | 0 )
#define _CONTROL      ( column6 | 1 )
#define _GRAPH        ( column6 | 2 )
#define _CAPS         ( column6 | 3 )
#define _CODE         ( column6 | 4 )
#define _F1           ( column6 | 5 )
#define _F2           ( column6 | 6 )
#define _F3           ( column6 | 7 )

#define _F4           ( column7 | 0 )
#define _F5           ( column7 | 1 )
#define _ESC          ( column7 | 2 )
#define _TAB          ( column7 | 3 )
#define _STOP         ( column7 | 4 )
#define _BACKSPACE    ( column7 | 5 )
#define _SELECT       ( column7 | 6 )
#define _ENTER        ( column7 | 7 )

#define _SPACE        ( column8 | 0 )
#define _HOME         ( column8 | 1 )
#define _INSERT       ( column8 | 2 )
#define _DELETE       ( column8 | 3 )
#define _LEFT         ( column8 | 4 )
#define _UP           ( column8 | 5 )
#define _DOWN         ( column8 | 6 )
#define _RIGHT        ( column8 | 7 )

#define _KPPLUS       ( _NONE )    
#define _KPMINUS      ( _NONE )    
#define _KPTIMES      ( _NONE )    
#define _KPSLASH      ( _NONE )    

#define _KP1          ( _1       )    
#define _KP2          ( _2       )    
#define _KP3          ( _3       )    
#define _KP4          ( _4       )    

#define _KP5          ( _5       )    
#define _KP6          ( _6       )    
#define _KP7          ( _7       )    
#define _KP8          ( _8       )   
#define _KP9          ( _9       )       
#define _KPCOMMA      ( _COMMA       )    
#define _KPDOT        ( _DOT       )    



// Shifted
#define _F6           ( _F1 )        // F6
#define _F7           ( _F2 )        // F7
#define _F8           ( _F3 )        // F8
#define _F9           ( _F4 )        // F9
#define _F10          ( _F5 )        // F10

#define _EXCLAMATION  ( _1  )          // !
#define _QUOTE        ( _APOSTROPHE  ) // "
#define _NUMBER       ( _3  )          // #
#define _DOLLAR       ( _4  )          // $
#define _PERCENT      ( _5  )          // %
#define _POWER        ( _6  )          // ^
#define _AMPERSAND    ( _7  )          // &
#define _ASTERISK     ( _8  )          // *
#define _OPENBRACE    ( _9  )          // (
#define _CLOSEBRACE   ( _0  )          // )
#define _PIPE         ( _BACKSLASH )   // |

#define _AT           ( _2   )          // @
#define _UNDERSCORE   ( _MINUS      )   // _
#define _PLUS         ( _EQUAL      )   // +
#define _CLOSEKEY     ( _CLOSEBRACKET ) // }

#define _TILDE        ( _ACUTE      ) // ~
#define _OPENKEY      ( _OPENBRACKET) // ]

#define _LESSTHAN     ( _COMMA      ) // <
#define _GREATERTHAN  ( _DOT        ) // >
#define _COLON        ( _SEMICOLON  ) // :
#define _QUESTION     ( _SLASH      ) // ?


