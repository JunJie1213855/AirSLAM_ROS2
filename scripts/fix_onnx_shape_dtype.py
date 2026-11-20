#!/usr/bin/env python3
"""Convert int32 shape constants to int64 so TensorRT's strict type checks pass."""
import argparse
import numpy as np
import onnx
from onnx import TensorProto, numpy_helper


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    args = ap.parse_args()

    model = onnx.load(args.input, load_external_data=False)
    g = model.graph
    n = 0
    new_inits = []
    for init in g.initializer:
        if init.data_type == TensorProto.INT32:
            arr = numpy_helper.to_array(init).astype(np.int64)
            new_inits.append(numpy_helper.from_array(arr, init.name))
            n += 1
        else:
            new_inits.append(init)
    del g.initializer[:]
    g.initializer.extend(new_inits)
    onnx.checker.check_model(model, full_check=False)
    onnx.save(model, args.output)
    print(f"converted {n} int32 initializers -> int64: {args.output}")


if __name__ == "__main__":
    main()
