def register(device_name, ctx):
    return ('mock', MockReader(device_name, ctx))


class MockReader:
    def __init__(self, device_name, ctx):
        self.device_name = device_name
        self.ctx = ctx
