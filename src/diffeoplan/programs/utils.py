

class Storage:
    commands = {}


def declare_command(name, short_usage=None):
    if short_usage is None:
        short_usage = name
    def wrap(f):
        f.short_usage = short_usage
        if name in Storage.commands:
            raise Exception('Already defined command %r.' % name)
        Storage.commands[name] = f
        return f
    return wrap
