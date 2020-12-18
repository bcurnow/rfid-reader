from pkgutil import iter_modules
from importlib import import_module


def load_impl(reader_type, config):
    """ Find all the modules in the impl package and build a map based on the their type spec and class."""
    import rfidreader.impl as pkg

    for module_info in iter_modules(path=pkg.__path__):
        if module_info.name == reader_type:
            module = import_module(f'{pkg.__name__}.{module_info.name}')
            register_method = getattr(module, 'register', None)
            if register_method:
                return register_method(config)
