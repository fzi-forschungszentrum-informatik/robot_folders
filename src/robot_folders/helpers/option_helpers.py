import click


class SwallowAllOption(click.Option):
    """Option that swallows all values after it"""

    def __init__(self, *args, **kwargs):
        nargs = kwargs.pop("nargs", -1)

        if nargs != -1:
            raise ValueError("nargs has to be -1")

        super(SwallowAllOption, self).__init__(*args, **kwargs)

    def add_to_parser(self, parser, ctx):
        def parser_process(value, state):
            value = [value] + state.rargs
            value = tuple(value)
            state.rargs[:] = []

            self._previous_parser_process(value, state)

        retval = super(SwallowAllOption, self).add_to_parser(parser, ctx)
        for name in self.opts:
            our_parser = parser._long_opt.get(name) or parser._short_opt.get(name)
            if our_parser:
                self._eat_all_parser = our_parser
                self._previous_parser_process = our_parser.process
                our_parser.process = parser_process
                break
        return retval
