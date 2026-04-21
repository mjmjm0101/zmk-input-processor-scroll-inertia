# Contributing

This is a personal project, maintained on spare time.  Issues and
pull requests are read when I can get to them — responses may be
slow or absent.

If you do file one, please follow the format below.  Reports missing
key information are unlikely to get a useful response.

## Issues

Include all of:

- Hardware (trackball sensor, keyboard, ZMK board)
- ZMK version / west manifest revision
- The `&scroll_inertia` devicetree instance and the
  `input-processors` chain it sits in
- Expected vs. observed behaviour
- If tuning-related: which properties you changed from the defaults

## Pull requests

- Explain *why* in the commit message — the diff already shows what
- Run `make -C tests test` before submitting
- If you add or change a helper in `src/scroll_inertia_math.h`, add
  or update the corresponding test in `tests/test_math.c`
- If you change a tunable's semantics, update
  `dts/bindings/zmk,input-processor-scroll-inertia.yaml`
- If you rework the state machine, update the ASCII diagram at the
  top of `src/input_processor_scroll_inertia.c` and the explanation
  in `README.md` / `README_ja.md`

### Building locally

Math-only tests (no Zephyr toolchain needed):

```sh
cd tests
make test
```

Firmware build follows the standard ZMK external-module workflow;
see `README.md` for the recommended placement in the
`input-processors` chain.

## License

By contributing, you agree that your contributions will be licensed
under the MIT License (same as the rest of this repository).
