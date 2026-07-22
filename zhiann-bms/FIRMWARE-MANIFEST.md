# CubeOrangePlus firmware provenance

This manifest identifies the reviewed Zhiann BMS firmware artifact. The build
was made from a clean detached build-only commit because the reviewed changes
have not been committed to the main working branch.

## Source

- Base commit: `900cfe4445b920dba2a1d3868fe2329218d8ce6b`
- Detached build commit: `8ca23bbe5f0ac71cead632c305d0b3f5fef953d0`
- Embedded APJ identity: `8ca23bbe`
- SHA-256 of `git diff --binary 900cfe4445 8ca23bbe5f0a`:
  `70b2723f579f71cc7cfbcd20111faa55c7481246e05b7060df72ea22d9a5c24b`

The detached commit changes exactly the same tracked files and bytes as the
reviewed main worktree at build time. It does not move or commit the main
branch.

## Artifact

- File: `arducopter-CubeOrangePlus.apj`
- SHA-256: `a297ca1031b0a4485b5ba4db2b207d553dcf78dae8814a6b4f0eed82cd32fad7`
- APJ image size: 1,965,776 bytes
- Waf flash use: 1,965,772 of 1,966,080 bytes
- Flash free: **304 bytes**
- Board ID: 1063 (`CubeOrangePlus`)

The build was run twice from the same clean commit and produced a
byte-identical APJ. Embedded strings for all Zhiann warnings and the
`ZBMS`/`ZBC1`/`ZBC2` log messages were verified.

## Toolchain and command

- GNU Arm Embedded Toolchain: `10-2020-q4-major`
- `arm-none-eabi-g++`: 10.2.1 (20201103)

```sh
export PATH="/Users/mindprobe/ardupilot-tools/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf distclean
./waf configure --board CubeOrangePlus
./waf copter -j4
shasum -a 256 build/CubeOrangePlus/bin/arducopter.apj
```

## Release caveat

The artifact fits, but 304 bytes is critically low headroom. Any source,
compiler, feature, or board-definition change requires a fresh size-checked
build. Establish a larger flash budget before upstreaming or extending the
driver.
