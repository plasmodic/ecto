# Directive that shows a toggle link between two sections
# The first one is initially showed. Here is the syntax:
# .. toggle1:: click here for ROS instructions
#
#    some RST text
#
#.. toggle2:: click here for non-ROS instructions
#
#    some more RST text
import docutils
from docutils import nodes
from docutils.parsers.rst import directives, roles, states

def setup(app):
    app.add_directive('toggle1', ToggleDirective1)
    app.add_directive('toggle2', ToggleDirective2)

class ToggleDirective(docutils.parsers.rst.Directive):
    """
    Base class that will be used by the two toggle directives
    """                                                                                   
    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = True
    option_spec = {}
    has_content = True
    node_class = nodes.container

    def run_help(self, id):
        self.assert_has_content()
        text = '\n'.join(self.content)
        # Create the node, to be populated by `nested_parse`.
        node = self.node_class(rawsource=text)
        node += nodes.raw(self.arguments[0], '<a class="toggleable_link_%s" href="javascript:toggle();">%s</a>' % (id, self.arguments[0]), format="html")
        node += nodes.raw(self.arguments[0], '<div class="toggleable_div_%s">' % id, format="html")

        # Parse the directive contents.
        self.state.nested_parse(self.content, self.content_offset, node)
        node += nodes.raw(self.arguments[0], '</div>', format="html")
        return [node]

class ToggleDirective1(ToggleDirective):
    def run(self):
        return self.run_help(1)

class ToggleDirective2(ToggleDirective):
    def run(self):
        return self.run_help(2)
