from ..ply import lex

reserved = {
    'xposition': 'XPOSITION',
    'yposition': 'YPOSITION',
    'zposition': 'ZPOSITION',
    'xrotation': 'XROTATION',
    'yrotation': 'YROTATION',
    'zrotation': 'ZROTATION',
    'hierarchy': 'HIERARCHY',
    'root': 'ROOT',
    'offset': 'OFFSET',
    'channels': 'CHANNELS',
    'joint': 'JOINT',
    'motion': 'MOTION'
}

tokens = [
             'ENDJOINT', 'FRAMES', 'FRAMETIME', 'NAME', 'FNUMBER',
             'NUMBER', 'LPAREN', 'RPAREN'
         ] + list(reserved.values())


def BVHLexer():
    is_in_motion_section = False
    # Tokens

    # Modified tokens

    t_LPAREN = r'{'
    t_RPAREN = r'}'

    def t_ENDJOINT(t):
        r'(?i)END\ +SITE'
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

    def t_NAME(t):
        r'[a-zA-Z_0-9-_.]+'
        t.type = reserved.get(t.value.lower(), 'ID')  # Check for reserved words
        if t.type == "MOTION":
            nonlocal is_in_motion_section
            is_in_motion_section = True
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
