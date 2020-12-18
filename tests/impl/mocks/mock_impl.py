def register(config):
    return MockReader(config)


class MockReader:
    def __init__(self, config):
        self.config = config
