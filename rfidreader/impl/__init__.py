from pkgutil import iter_modules
from importlib import import_module


def register_readers(device_name, ctx):
    """ Find all the modules in the impl package and build a map based on the their type spec and class."""
    import rfidreader.impl as pkg
    rv = {}

    for module_info in iter_modules(path=pkg.__path__):
        module = import_module(f'{pkg.__name__}.{module_info.name}')
        register_method = getattr(module, 'register', None)
        if register_method:
            type, instance = register_method(device_name, ctx)
            rv[type] = instance

    return rv
