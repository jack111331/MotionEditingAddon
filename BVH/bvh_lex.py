from ..ply import lex

tokens = (
    'XPOSITION', 'YPOSITION',
    'ZPOSITION', 'XROTATION', 'YROTATION', 'ZROTATION',
    'HIERARCHY', 'ROOT', 'OFFSET', 'CHANNELS', 'JOINT',
    'ENDJOINT', 'MOTION', 'FRAMES', 'FRAMETIME', 'NAME', 'FNUMBER'
     'NUMBER', 'LPAREN', 'RPAREN',
)
def BVHLexer():
    is_in_motion_section = False
    # Tokens

    # Modified tokens
    t_XPOSITION = r'(?i)XPOSITION'
    t_YPOSITION = r'(?i)YPOSITION'
    t_ZPOSITION = r'(?i)ZPOSITION'
    t_XROTATION = r'(?i)XROTATION'
    t_YROTATION = r'(?i)YROTATION'
    t_ZROTATION = r'(?i)ZROTATION'

    t_HIERARCHY = r'(?i)HIERARCHY'
    def t_ROOT(t):
        r'(?i)ROOT'
        return t

    t_OFFSET = r'(?i)OFFSET'
    t_CHANNELS = r'(?i)CHANNELS'
    t_JOINT = r'(?i)JOINT'
    t_ENDJOINT = r'(?i)END\ +SITE'

    t_LPAREN  = r'{'
    t_RPAREN  = r'}'
    t_NAME    = r'[a-zA-Z]+'

    def t_MOTION(t):
        r'(?i)MOTION'
        nonlocal is_in_motion_section
        is_in_motion_section = True
        return t

    def t_FRAMES(t):
        r'(?i)FRAMES\ *:'
        nonlocal is_in_motion_section
        if is_in_motion_section == False:
            print("[Error] Not in motion section")
        return t

    def t_FRAMETIME(t):
        r'(?i)FRAME\ +TIME\ *:'
        nonlocal is_in_motion_section
        if is_in_motion_section == False:
            print("[Error] Not in motion section")
        return t

    def t_FNUMBER(t):
        r'[\+\-]*\d+\.\d+'
        t.value = float(t.value)
        return t

    def t_NUMBER(t):
        r'[\+\-]*\d+'
        t.value = int(t.value)
        return t

    # Ignored characters
    t_ignore = " \t"

    def t_newline(t):
        r'\n+'
        t.lexer.lineno += t.value.count("\n")

    def t_error(t):
        print("At ", t.lexer.lineno)
        print(f"Illegal character {t.value[0]!r}")
        t.lexer.skip(1)

    return lex.lex()