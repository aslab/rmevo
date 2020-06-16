import argparse


class CustomParser(argparse.ArgumentParser):
    """
    Extends argument parser to add some simple file reading / writing
    functionality.
    """

    def convert_arg_line_to_args(self, arg_line):
        """
        Simple arg line converter that returns `--my-argument value` from
        lines like "my_argument=value"
        :param arg_line:
        :return:
        """
        # Empty or comment line
        if not arg_line or arg_line[0] == "#":
            return []

        split = arg_line.find("=")
        if split < 0:
            return [arg_line]

        k, v = "--" + arg_line[:split].replace("_", "-"), arg_line[1 + split:]

        # Try to determine if this key is a store constant action, if so
        # return only the key.
        const = False
        for a in self._actions:
            if k in a.option_strings and a.const is not None:
                const = True
                break

        return [k] if const else [k, v]

    @staticmethod
    def record(args, file):
        """
        Takes the result of `parse_args` and writes it back to a file.
        """
        lines = ["{key}={value}\n".format(key=k, value=args.__dict__[k]) for k
                 in sorted(args.__dict__.keys())]
        with open(file, 'w') as configuration_output:
            configuration_output.writelines(lines)


def str_to_bool(v):
    """
    :type v: str
    """
    return v.lower() in ("true", "1")


def str_to_address(v):
    """
    :type v: str
    """
    if not v:
        return None

    host, port = v.split(":", 1)
    return host, int(port)


parser = CustomParser(fromfile_prefix_chars='@')

parser.add_argument(
    '--manager',
    default=None,
    type=str,
    help="RMEvo manager"
)