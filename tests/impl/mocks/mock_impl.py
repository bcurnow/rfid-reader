def register():
    return ('mock', MockReader)


class MockReader:
    def __init__(self, config):
        self.config = config
