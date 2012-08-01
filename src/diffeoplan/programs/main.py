from . import Storage, logger
from ..utils import (CmdOptionParser, MyOptionParser, UserError,
    wrap_script_entry_point)
from bootstrapping_olympics.utils import substitute
from conf_tools import ConfToolsException
import contracts
from diffeoplan.configuration.master import DiffeoplanConfig

MAIN_CMD_NAME = 'dp'
commands_list = "\n".join(['%25s   %s' % 
                           #(f.short_usage, f.__doc__)
                           (cmd, str(f.__doc__).strip())
                           for cmd, f in Storage.commands.items()])

usage_pattern = """

    ${cmd} [global options]  <command>  [command options]
    
Available commands:
${commands_list}

Use: `${cmd}  <command> -h' to get more information about that command.  
"""


def dp(arguments):
    usage = substitute(usage_pattern, commands_list=commands_list,
                       cmd=MAIN_CMD_NAME)

    parser = MyOptionParser(prog=MAIN_CMD_NAME, usage=usage)
    parser.disable_interspersed_args()

    parser.add_option("--contracts", default=False, action='store_true',
                      help="Activate PyContracts (disabled by default)")

    (options, args) = parser.parse_args(arguments)

    if not options.contracts:
        logger.warning('PyContracts disabled for speed. Use --contracts to activate.')
        contracts.disable_all()

    if not args:
        msg = ('Please supply a command.\nAvailable commands:\n%s' % commands_list)
        raise UserError(msg)

    cmd = args[0]
    cmd_args = args[1:]

    if not cmd in Storage.commands:
        msg = ('Unknown command %r. Available: %s.' % 
               (cmd, ", ".join(Storage.commands.keys())))
        raise UserError(msg)

    confdir = '.'
    DiffeoplanConfig.load(confdir)

    
    function = Storage.commands[cmd]
    usage = function.short_usage 
    parser = CmdOptionParser(prog='%s %s' % (MAIN_CMD_NAME, cmd), usage=usage,
                             args=cmd_args)
    return function({}, parser)


def dpmain():
    exceptions_no_traceback = (UserError, ConfToolsException)
    return wrap_script_entry_point(dp, logger,
                        exceptions_no_traceback=exceptions_no_traceback)


if __name__ == '__main__':
    dpmain()

