from pysmt.shortcuts import Symbol

class SymbolArray:
    def __init__(self, name, shape):
        self.name = name
        self.dim = 1 if type(shape) == int else len(shape)
        self.arr = {}
        self.used_mapping = {}

    def __getitem__(self, idx):
        if type(idx) == int:
            idx = (idx,)
        assert len(idx) == self.dim
        if idx not in self.arr:
            self.arr[idx] = Symbol(self.get_symbol_name(idx))
        self.used_mapping[self.arr[idx]] = (self.name, idx)
        return self.arr[idx]

    def get_symbol_name(self, idx):
        return self.name + "(" + ",".join(map(lambda x: str(x), idx)) + ")"

    def get_used_keys(self):
        return set(self.used_mapping.keys())
