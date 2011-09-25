
from pygments.lexer import RegexLexer
from pygments.token import *

class EctoShLexer(RegexLexer):
    name = 'Better lexer sh examples'
    aliases = ['ectosh']

    tokens = {
        'root': [
            (r'^(%|\>\>\>|\(gdb\))', Literal.Number.Float, 'afterprompt'),
            (r'.+', Text),
        ],
        'afterprompt': [
            (r'.*', Generic.Deleted, '#pop'),
            (r'\n', Comment.Multiline, '#pop'),
        ]
    }

def setup(app):
    app.add_lexer('ectosh', EctoShLexer())


