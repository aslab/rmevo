#!/usr/bin/env python3
import os
import sys
import asyncio
import importlib

from pyrmevo import parser

here = os.path.dirname(os.path.abspath(__file__))
rvpath = os.path.abspath(os.path.join(here, '..', 'revolve'))
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


def run(loop, arguments):
    if arguments.manager is not None:
        # this split will give errors on windows
        manager_lib = os.path.splitext(arguments.manager)[0]
        manager_lib = '.'.join(manager_lib.split('/'))
        print(manager_lib)
        #manager = importlib.import_module(manager_lib,'pyrmevo.tutorials').run
        manager = importlib.import_module('.config_evo' + manager_lib, package='pyrmevo').run
        return loop.run_until_complete(manager())
    else:
        assert ValueError("RMEvo needs manager argument")


def main():
    import traceback

    def handler(_loop, context):
        try:
            exc = context['exception']
        except KeyError:
            print(context['message'])
            return

        if isinstance(exc, ConnectionResetError):
            print("Got disconnect / connection reset - shutting down.")
            sys.exit(0)

        if isinstance(exc, OSError) and exc.errno == 9:
            print(exc)
            traceback.print_exc()
            return

        # traceback.print_exc()
        raise exc

    try:
        arguments = parser.parse_args()
        loop = asyncio.get_event_loop()
        loop.set_exception_handler(handler)
        run(loop, arguments)
    except KeyboardInterrupt:
        print("Got CtrlC, shutting down.")


if __name__ == '__main__':
    print("STARTING")
    main()
    print("FINISHED")

