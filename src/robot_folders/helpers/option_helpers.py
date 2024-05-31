import click


# stolen from https://stackoverflow.com/questions/48391777/nargs-equivalent-for-options-in-click/48394004#48394004
class OptionEatAll(click.Option):
    """Option that can have an infite number of arguments"""

    def __init__(self, *args, **kwargs):
        """
        - **parameters** and **types**::
            :param args: Variable length argument list, passed to Option constructor
            :param kwargs: Variable length keyword argument list, passed to Option constructor
            :param save_other_options: If set to true, stops parsing if another if another argument
                                       prefix is encountered (e.g. something starting with a double
                                       dash like --foo)
            :type save_other_options: bool
        """
        self.save_other_options = kwargs.pop("save_other_options", True)
        nargs = kwargs.pop("nargs", -1)
        assert nargs == -1, "nargs, if set, must be -1 not {}".format(nargs)
        super(OptionEatAll, self).__init__(*args, **kwargs)
        self._previous_parser_process = None
        self._eat_all_parser = None

    def add_to_parser(self, parser, ctx):

        def parser_process(value, state):
            # method to hook to the parser.process
            done = False
            value = [value]
            if self.save_other_options:
                # grab everything up to the next option
                while state.rargs and not done:
                    for prefix in self._eat_all_parser.prefixes:
                        if state.rargs[0].startswith(prefix):
                            done = True
                    if not done:
                        value.append(state.rargs.pop(0))
            else:
                # grab everything remaining
                value += state.rargs
                state.rargs[:] = []
            value = tuple(value)

            # call the actual process
            self._previous_parser_process(value, state)

        retval = super(OptionEatAll, self).add_to_parser(parser, ctx)
        for name in self.opts:
            our_parser = parser._long_opt.get(name) or parser._short_opt.get(name)
            if our_parser:
                self._eat_all_parser = our_parser
                self._previous_parser_process = our_parser.process
                our_parser.process = parser_process
                break
        return retval
