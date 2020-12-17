import os
import pkgutil
import sys
import importlib.util

from unittest.mock import call, patch

from rfidreader.impl import register_readers


@patch('rfidreader.impl.import_module')
@patch('rfidreader.impl.iter_modules')
def test_register_readers(iter_modules, import_module):
    import rfidreader.impl as real_pkg
    mocks = import_mocks()
    import mocks.mock_impl
    import mocks.not_an_impl
    iter_modules.return_value = pkgutil.iter_modules(path=mocks.__path__)
    import_module.side_effect = [mocks.mock_impl, mocks.not_an_impl]
    readers = register_readers('test', {})
    assert len(readers) == 1
    assert 'mock' in readers
    reader = readers['mock']
    assert reader.device_name == 'test'
    assert reader.ctx == {}
    iter_modules.assert_called_once_with(path=real_pkg.__path__)
    import_module.assert_has_calls([
        call(f'{real_pkg.__name__}.mock_impl'),
        call(f'{real_pkg.__name__}.not_an_impl'),
    ], any_order=False)


def import_mocks():
    # Because of the way python works, we need to do a bit of trickery to make the mocks package available
    # First, determine the absolute path to the mocks directory
    module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'mocks'))

    # Import the __init__.py file as a module to serve as the package
    package = load_from_path('mocks', os.path.join(module_path, '__init__.py'))

    return package


def load_from_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module
