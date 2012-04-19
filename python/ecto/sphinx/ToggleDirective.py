# Directive that shows a toggle link between sections.
# Those sections cannot contain titles
# The first one is initially showed. Here is the syntax:
# .. toggle_table::
#     :arg1: text in button 1
#     :arg2: text in button 2
#
# .. toggle:: text in button 1
#
#    some RST text
#
#.. toggle:: text in button 2
#
#    some more RST text
import docutils
from docutils import nodes
from docutils.parsers.rst import directives, roles, states

def setup(app):
    app.add_directive('toggle', ToggleDirective)
    app.add_directive('toggle_table', ToggleTableDirective)

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

    def run(self):
        self.assert_has_content()
        text = '\n'.join(self.content)
        # Create the node, to be populated by `nested_parse`.
        node = self.node_class(rawsource=text)
        label = self.arguments[0]
        label_strip = label.replace(' ', '')
        node += nodes.raw(self.arguments[0], '<div class="toggleable_div label_%s">' % label_strip, format="html")

        # Parse the directive contents.
        self.state.nested_parse(self.content, self.content_offset, node)
        node += nodes.raw(self.arguments[0], '</div>', format="html")
        return [node]

class ToggleTableDirective(docutils.parsers.rst.Directive):
    """
    Class used to create a set of buttons to toggle different sections
    """
    required_arguments = 0
    optional_arguments = 10
    final_argument_whitespace = True
    option_spec = {}
    for i in xrange(0, 100):
        option_spec['arg' + str(i)] = str
    has_content = True
    node_class = nodes.container

    def run(self):
        # Create the node, to be populated by `nested_parse`.
        node = self.node_class()
        for key in self.options.iterkeys():
            if key not in self.option_spec:
                raise RuntimeError(key + ' not in the contructor of ToggleTableDirective, use arg0 to arg99')
            label = self.options[key]
            label_strip = label.replace(' ', '')
            node += nodes.raw(key, '<button class="toggleable_button label_%s" onClick="javascript:toggle(\'%s\');">%s</button>' %
                              (label_strip, label_strip, label), format="html")

        return [node]
