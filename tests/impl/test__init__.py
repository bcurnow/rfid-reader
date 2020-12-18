import os
import pkgutil
import sys
import importlib.util

import pytest
from unittest.mock import patch

from rfidreader.impl import load_impl


@pytest.mark.parametrize(
    ('reader_type', 'import_module_return_value', 'expected_reader_class'),
    [

        ('mock_impl', ('mock_impl',), ('mock_impl', 'MockReader')),
        ('unknown_type', (), None),
        ('not_an_impl', ('not_an_impl',), None),
    ],
    ids=['found', 'module notfound', 'found no register']
    )
@patch('rfidreader.impl.import_module')
@patch('rfidreader.impl.iter_modules')
def test_load_impl(iter_modules, import_module, reader_type, import_module_return_value, expected_reader_class):
    # Make sure that the modules under mocks are imported
    from mocks import mock_impl, not_an_impl  # noqa: F401
    import rfidreader.impl as real_pkg
    iter_modules.return_value = pkgutil.iter_modules(path=mocks.__path__)
    import_module.return_value = get_by_name(mocks, import_module_return_value)
    reader = load_impl(reader_type, {})
    if expected_reader_class:
        assert isinstance(reader, get_by_name(mocks, expected_reader_class))
    else:
        assert reader is None
    iter_modules.assert_called_once_with(path=real_pkg.__path__)
    if expected_reader_class:
        import_module.assert_called_once_with(f'{real_pkg.__name__}.{reader_type}')


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


# Only import it once or the code won't work
mocks = import_mocks()


def get_by_name(base, names):
    rv = base
    for name in names:
        rv = getattr(rv, name)

    return rv
